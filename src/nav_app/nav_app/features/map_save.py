"""Save request lifecycle, independent of ROS transport and preview state."""
from threading import RLock
from .map_commit import committed_map_receipt, receipt_snapshot


class MapSaveLifecycle:
    def init_save_lifecycle(self):
        self._save_lock = RLock()
        self._save_future = None
        self._save_attempt = None
        self.save_attempt_id = 0
        self.last_save_result = None
        self._save_used_paths = set()

    def begin_save(self, output_path, *, native_commit=False, legacy_root=None):
        with self._save_lock:
            if self.save_in_progress:
                raise ValueError('上一次地图仍在保存，请稍候')
            sidecar = output_path.with_name(output_path.name + '.commit.json')
            if (output_path in self._save_used_paths or output_path.exists() or output_path.is_symlink()
                    or sidecar.exists() or sidecar.is_symlink()):
                raise ValueError('地图保存目标已存在，请重新选择名称')
            prior_receipts = receipt_snapshot(output_path, legacy_root) if native_commit else {}
            self.save_attempt_id += 1
            self._save_used_paths.add(output_path)
            self._save_attempt = dict(id=self.save_attempt_id, path=output_path,
                prior_receipts=prior_receipts, native_commit=native_commit, legacy_root=legacy_root)
            self._save_future = None
            self._pending_save_path = output_path
            self.save_in_progress = True
            self.save_status = f'正在保存：{output_path.name}'
            return self.save_attempt_id

    def watch_save(self, future, output_path=None, attempt_id=None):
        with self._save_lock:
            if (not self.save_in_progress or (output_path is not None and output_path != self._pending_save_path)
                    or (attempt_id is not None and attempt_id != self.save_attempt_id)):
                future.cancel()
                return
            self._save_future = future
            future.add_done_callback(self._on_save_done)

    def fail_save_call(self, attempt_id, reason):
        with self._save_lock:
            if self.save_in_progress and self.save_attempt_id == attempt_id:
                self.cancel_pending_save(reason)

    def _committed_receipt(self):
        attempt = self._save_attempt
        if not attempt or not attempt['native_commit']:
            return None
        return committed_map_receipt(attempt['path'], prior_receipts=attempt['prior_receipts'],
                                     legacy_root=attempt['legacy_root'])

    def _finish_save(self, success, status, evidence=None):
        self.last_save_result = dict(attempt_id=self.save_attempt_id, success=success,
            path=str(self._pending_save_path) if success else None, evidence=evidence)
        if success:
            self.last_saved_path = self._pending_save_path
        self.save_status = status
        self._save_future = None
        self._save_attempt = None
        self.save_in_progress = False
        self._pending_save_path = None
        self.save_revision += 1

    def mapping_exited(self, reason):
        """Consume a completed ACK or durable receipt before invalidating it.

        Caller must wait for the mapper to exit: its error signal can arrive
        while a save worker is still committing the file.
        """
        with self._save_lock:
            if not self.save_in_progress:
                return
            future = self._save_future
            if future is not None and future.done() and not future.cancelled():
                self._on_save_done(future)
                return
            receipt = self._committed_receipt()
            if receipt:
                self._finish_save(True, f'地图已保存，建图服务已退出：{self._pending_save_path}', receipt)
                if future is not None:
                    future.cancel()
                return
            self.cancel_pending_save(reason)

    def cancel_pending_save(self, reason):
        with self._save_lock:
            if not self.save_in_progress:
                return
            future = self._save_future
            self._finish_save(False, reason)
            # Invalidate identity before cancellation, which may run callbacks.
            if future is not None:
                future.cancel()

    def _on_save_done(self, future):
        with self._save_lock:
            # A response from the previous mapper must not complete a new save.
            if future is not self._save_future:
                return
            output_path = self._pending_save_path
            success, evidence = False, None
            try:
                response = future.result()
                if (response is not None and response.success and output_path is not None
                        and output_path.is_file() and output_path.stat().st_size > 0):
                    success, evidence = True, dict(source='service_response')
                    receipt = self._committed_receipt()
                    if receipt:
                        evidence.update(receipt, source='service_response', commit_source=receipt['source'])
                    status = f"地图保存完成：{output_path}"
                elif response is not None and response.success:
                    status = f"建图服务返回成功，但没有找到有效文件：{output_path}"
                else:
                    status = f"建图服务返回保存失败：{getattr(response, 'message', '')}"
            except Exception as exc:
                status = f"地图保存失败：{exc}"
            if not success:
                receipt = self._committed_receipt()
                if receipt:
                    success, evidence = True, dict(receipt, response_error=status)
                    status = f'地图已保存；服务回执异常：{output_path}'
            self._finish_save(success, status, evidence)
