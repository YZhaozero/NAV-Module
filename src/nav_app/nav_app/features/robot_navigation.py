"""Robot motion action lifecycle, isolated from pedestrian route planning."""
import time


class RobotNavigation:
    def __init__(self, client, changed, clock=time.monotonic, ready=None):
        self.client, self.changed, self.clock = client, changed, clock
        self.ready = ready
        self.generation = 0
        self.phase = "idle"
        self.status = "等待确认机器人路线"
        self.handle = None
        self.result = None
        self.result_revision = 0
        self.deadline = None
        self.cancel_requested = False

    @property
    def busy(self):
        return self.phase in ("submitting", "running", "cancelling", "cancel_failed")

    def notify(self, text):
        self.status = text
        self.changed(self)

    def start(self, goal):
        if self.busy:
            self.notify("当前任务尚未结束，请先取消并等待确认")
            return False
        if not self.client.server_is_ready() or self.ready is not None and not self.ready():
            self.notify("机器人导航服务未就绪")
            return False
        self.generation += 1
        token = self.generation
        self.handle, self.result = None, None
        self.cancel_requested = False
        self.phase, self.deadline = "submitting", self.clock() + 8.
        self.notify("正在提交机器人任务")
        try:
            self.client.send_goal_async(goal, feedback_callback=lambda reply: self._feedback(token, reply)).add_done_callback(
                lambda reply: self._accepted(token, reply))
        except Exception as exc:
            self.finish(token, -1, f"任务提交失败：{exc}")
            return False
        return True

    def _accepted(self, token, future):
        try:
            handle = future.result()
        except Exception as exc:
            if token == self.generation and self.busy:
                # An interrupted response does not prove the remote goal was rejected.
                self.phase, self.deadline = "cancel_failed", None
                self.cancel_requested = True
                self.notify(f"任务接收状态未确认，请检查机器人连接：{exc}")
            return
        if token != self.generation or not self.busy:
            if handle.accepted:
                try:
                    handle.cancel_goal_async()
                except Exception:
                    pass
            return
        if not handle.accepted:
            self.finish(token, -2, "机器人拒绝了路线，请检查定位与可达性")
            return
        self.handle, self.deadline = handle, None
        self.phase = "cancelling" if self.cancel_requested else "running"
        try:
            handle.get_result_async().add_done_callback(lambda reply: self._result(token, reply))
        except Exception as exc:
            self.phase = "cancel_failed"
            self.notify(f"任务结果不可读，请取消并确认：{exc}")
            return
        if self.cancel_requested:
            self.cancel()
        elif self.phase == "running":
            self.notify("机器人正在导航")

    def _feedback(self, token, reply):
        if token == self.generation and self.phase == "running":
            self.notify(f"机器人导航中 · 剩余 {reply.feedback.number_of_poses_remaining} 个目标")

    def _result(self, token, future):
        if token != self.generation or not self.busy:
            return
        try:
            status = future.result().status
        except Exception as exc:
            self.phase = "cancel_failed"
            self.notify(f"结束状态未确认，请重试取消：{exc}")
            return
        if status not in (4, 5, 6):
            self.phase = "cancel_failed"
            self.notify("尚未收到机器人结束状态，请重试取消")
            return
        self.finish(token, status, {4: "机器人已到达目的地", 5: "机器人任务已取消", 6: "机器人导航失败"}[status])

    def finish(self, token, result, text):
        if token != self.generation or not self.busy:
            return
        self.phase, self.result = "finished", result
        self.handle, self.deadline = None, None
        self.result_revision += 1
        self.notify(text)

    def cancel(self):
        if not self.busy:
            return False
        self.cancel_requested = True
        self.phase, self.deadline = "cancelling", self.clock() + 8.
        self.notify("正在取消机器人任务 · 等待确认")
        if self.handle is not None:
            token = self.generation
            try:
                self.handle.cancel_goal_async().add_done_callback(lambda reply: self._cancel_reply(token, reply))
                self.handle.get_result_async().add_done_callback(lambda reply: self._result(token, reply))
            except Exception as exc:
                self.phase = "cancel_failed"
                self.notify(f"取消请求失败，可重试：{exc}")
        return True

    def _cancel_reply(self, token, future):
        if token != self.generation or not self.busy:
            return
        try:
            reply = future.result()
            accepted = reply.return_code == 0 and bool(reply.goals_canceling)
        except Exception as exc:
            self.phase = "cancel_failed"
            self.notify(f"取消未确认，请重试：{exc}")
            return
        if not accepted:
            self.phase = "cancel_failed"
            self.notify("取消未获确认，请重试或检查机器狗")
        else:
            self.notify("取消请求已接受 · 等待机器人任务结束")

    def poll(self):
        if self.deadline is not None and self.clock() > self.deadline and self.busy:
            self.deadline = None
            self.cancel_requested = True
            self.phase = "cancel_failed"
            self.notify("机器人任务状态确认超时，请重试取消")
