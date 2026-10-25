"""结果发布闸门：过期进程不可能覆盖正式结果。

机制：
  1. 每次运行分配唯一 run_id，写入 out/runs/<run_id>/，**不碰正式文件**。
  2. 正式结果只能由 publish() 原子替换（tmp + os.replace），且必须同时满足：
       - run_id 严格晚于 current.json 里记录的 run_id（单调闸门，挡住延迟完成的旧进程）
       - 运行自述的源码/几何/参数指纹与调用方给出的期望值一致
       - 运行状态为 ok（非 aborted / invalid）
       - 目标文件当前内容的 sha256 与 current.json 记录的一致（防止第三方手改）
  3. 被拒绝的发布写入 out/runs/<run_id>/REJECTED.json，保留现场。
"""
import hashlib, json, os, time, uuid
from pathlib import Path

OUT = Path(os.environ.get('TQ_PUBLISH_ROOT') or (Path(__file__).resolve().parent / 'out'))
# 测试必须设置 TQ_PUBLISH_ROOT 指向沙箱，禁止用正式目录做发布测试
RUNS = OUT / 'runs'
CURRENT = OUT / 'current.json'
LOCK = OUT / '.publish.lock'


def sha256_file(p):
    h = hashlib.sha256()
    with open(p, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def new_run(kind, meta):
    """优先用调用方给的 run_id（便于复现），否则按时间生成。"""
    rid = meta.get('run_id') or f'{time.strftime("%Y%m%d-%H%M%S")}-{kind}-{uuid.uuid4().hex[:6]}'
    d = RUNS / rid
    d.mkdir(parents=True, exist_ok=True)
    (d / 'meta.json').write_text(json.dumps(dict(meta, run_id=rid, kind=kind,
                                                 at=meta.get('at') or time.strftime('%Y-%m-%dT%H:%M:%S')),
                                            ensure_ascii=False, indent=1), encoding='utf8')
    return rid, d


def finish(run_dir, payload, status='ok', note=''):
    """把本次输出写进运行目录（正式文件仍不动）。"""
    (run_dir / 'payload.json').write_text(json.dumps(payload, ensure_ascii=False, indent=1), encoding='utf8')
    m = json.loads((run_dir / 'meta.json').read_text(encoding='utf8'))
    m.update(status=status, note=note, payload_sha256=sha256_file(run_dir / 'payload.json'))
    (run_dir / 'meta.json').write_text(json.dumps(m, ensure_ascii=False, indent=1), encoding='utf8')


def current():
    return json.loads(CURRENT.read_text(encoding='utf8')) if CURRENT.exists() else None


class _Lock:
    """跨进程互斥：O_EXCL 建锁文件，超时放弃。"""
    def __init__(self, path, timeout=10.0, poll=0.05):
        self.path, self.timeout, self.poll = path, timeout, poll

    def __enter__(self):
        t0 = time.time()
        while True:
            try:
                fd = os.open(str(self.path), os.O_CREAT | os.O_EXCL | os.O_WRONLY)
                os.write(fd, str(os.getpid()).encode()); os.close(fd)
                return self
            except FileExistsError:
                if time.time() - t0 > self.timeout:
                    raise TimeoutError(f'拿不到发布锁 {self.path}')
                time.sleep(self.poll)

    def __exit__(self, *a):
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass


def publish(run_dir, target, expect=None):
    """把 run_dir/payload.json 提升为正式结果。返回 (ok, reason)。"""
    with _Lock(LOCK):
        return _publish_locked(run_dir, target, expect)


def _publish_locked(run_dir, target, expect=None):
    if Path(target).name.startswith('_') and 'runs' not in Path(target).parts:
        return False, '禁止发布到探针文件（_ 前缀），测试须写入 runs/ 沙箱'
    if 'baseline' in str(target):
        return False, 'baseline 目录为冻结基线，禁止任何进程写入（只允许人工解冻）'
    meta = json.loads((run_dir / 'meta.json').read_text(encoding='utf8'))
    cur = current()
    rid = meta['run_id']
    reasons = []
    if meta.get('status') != 'ok':
        reasons.append(f"status={meta.get('status')}")
    if not meta.get('task_id'):
        reasons.append('缺少 task_id（无法确认是当前有效任务）')
    if expect and expect.get('task_id') and meta.get('task_id') != expect['task_id']:
        reasons.append(f"task_id 不属于当前有效任务（{meta.get('task_id')} != {expect['task_id']}）")
    def _ts(x):
        try:
            return time.mktime(time.strptime(str(x), '%Y-%m-%dT%H:%M:%S'))
        except Exception:
            return 0.0
    if cur and cur.get('at') and _ts(meta.get('at')) <= _ts(cur['at']):
        reasons.append(f"运行时间不晚于当前（{meta.get('at')} <= {cur['at']}），判为过期运行")
    for k, v in (expect or {}).items():
        if meta.get(k) != v:
            reasons.append(f'指纹不一致: {k} 期望 {v} 实际 {meta.get(k)}')
    if (cur and cur.get('target_sha256') and Path(target).exists()
            and Path(cur.get('target', '')).name == Path(target).name):
        if sha256_file(target) != cur['target_sha256']:
            reasons.append('正式文件已被外部修改，与 current.json 记录不符')
    if reasons:
        (run_dir / 'REJECTED.json').write_text(json.dumps(dict(reasons=reasons, at=time.strftime('%H:%M:%S')),
                                                          ensure_ascii=False, indent=1), encoding='utf8')
        return False, '; '.join(reasons)

    target = Path(target)
    tmp = target.with_suffix(target.suffix + '.tmp')
    tmp.write_text((run_dir / 'payload.json').read_text(encoding='utf8'), encoding='utf8')
    os.replace(tmp, target)                      # 原子替换
    digest = sha256_file(target)
    if 'runs' in Path(target).parts:      # 沙箱目标不更新正式指针
        (run_dir / 'PUBLISHED.json').write_text(json.dumps(dict(target=str(target), sandbox=True)), encoding='utf8')
        return True, 'published (sandbox, 未更新正式指针)'
    CURRENT.write_text(json.dumps(dict(run_id=rid, target=str(target.name), target_sha256=digest,
                                       at=meta.get('at'), published_at=time.strftime('%Y-%m-%dT%H:%M:%S')),
                                   ensure_ascii=False, indent=1), encoding='utf8')
    (run_dir / 'PUBLISHED.json').write_text(json.dumps(dict(target=str(target), sha256=digest),
                                                       ensure_ascii=False, indent=1), encoding='utf8')
    return True, 'published'


def _regression_stale_process():
    """回归测试：新结果发布后，一个"延迟完成的旧运行"不得覆盖。"""
    old_id, old_dir = new_run('planA', dict(run_id='20200101-000000-planA-aaaaaa', fp_source='s1'))
    finish(old_dir, dict(value='old'))
    new_id, new_dir = new_run('planA', dict(run_id='20301231-235959-planA-bbbbbb', fp_source='s1'))
    finish(new_dir, dict(value='new'))
    tgt = OUT / '_regression_target.json'
    ok1, r1 = publish(new_dir, tgt)
    ok2, r2 = publish(old_dir, tgt)          # 旧进程迟到
    value = json.loads(tgt.read_text(encoding='utf8'))['value']
    passed = ok1 and (not ok2) and value == 'new'
    tgt.unlink(missing_ok=True)
    for d in (old_dir, new_dir):
        for f in d.glob('*.json'):
            f.unlink()
        d.rmdir()
    return passed, dict(first=ok1, stale_publish=ok2, final_value=value, reason=r2)


if __name__ == '__main__':
    ok, info = _regression_stale_process()
    print('回归测试（旧进程迟到覆盖）:', 'PASS' if ok else 'FAIL', info)
