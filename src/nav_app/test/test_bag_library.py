import json
import yaml
from nav_app.features.bag_library import scan_bags


def bag(path):
    path.mkdir(parents=True)
    (path/'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information':{
        'duration':{'nanoseconds':125000000000}, 'message_count':500, 'topics_with_message_count':[]}}))
    (path/'segment.db3').write_bytes(b'test-payload')


def test_inventory_is_read_only_deduplicates_and_distinguishes_audit(tmp_path):
    folder=tmp_path/'recordings'/'capture'
    bag(folder)
    (folder/'nav_audit.json').write_text(json.dumps({'complete':True, 'errors':[]}))
    before={p:p.read_bytes() for p in folder.iterdir()}
    rows, notices=scan_bags([tmp_path,tmp_path/'recordings'])
    assert len(rows)==1 and not notices
    assert rows[0].duration_seconds==125 and rows[0].messages==500
    assert rows[0].status=='校验通过' and rows[0].can_load
    assert rows[0].size_bytes==sum(map(len,before.values()))
    assert before=={p:p.read_bytes() for p in folder.iterdir()}
    rows,_=scan_bags([tmp_path], folder)
    assert rows[0].status=='正在录制或整理' and not rows[0].can_load


def test_unfinished_and_unaudited_bags_remain_visible(tmp_path):
    bag(tmp_path/'ordinary')
    broken=tmp_path/'unfinished'
    broken.mkdir();(broken/'part.db3').write_bytes(b'payload')
    rows,_=scan_bags([tmp_path])
    states={r.path.name:r.status for r in rows}
    assert states=={'ordinary':'未校验','unfinished':'需要检查'}
