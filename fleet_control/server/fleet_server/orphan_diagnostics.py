"""Read-only orphan inventory; never repairs, deletes or registers bundles.

Run against a checkpointed offline database snapshot and quiescent bundle copy.
The SQLite immutable mode avoids creating WAL/SHM files and intentionally cannot
read live WAL contents. Observations alone are not an approval.
"""
import hashlib
import json
import os
import sqlite3
import stat
from urllib.parse import quote
from .map_registry import (MAX_YAML_BYTES, MAX_PGM_BYTES, MAX_POSEGRAPH_BYTES,
                           compute_bundle_sha256, make_map_version_id,
                           parse_pgm_p5, parse_ros_map_yaml, validate_entity_id,
                           validate_frame_id, SOURCE_MODES)

def _read_file(directory_fd, name, limit):
    fd = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=directory_fd)
    try:
        st = os.fstat(fd)
        if not stat.S_ISREG(st.st_mode) or st.st_size > limit:
            raise ValueError(name + ': non-regular file or size limit')
        with os.fdopen(fd, 'rb', closefd=False) as f: data = f.read(limit + 1)
        if len(data) > limit: raise ValueError(name + ': size limit')
        return data
    finally: os.close(fd)

def _hash_file(directory_fd, name, limit):
    fd = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=directory_fd)
    try:
        st = os.fstat(fd)
        if not stat.S_ISREG(st.st_mode) or st.st_size > limit:
            raise ValueError(name + ': non-regular file or size limit')
        digest=hashlib.sha256();total=0
        with os.fdopen(fd,'rb',closefd=False) as f:
            while True:
                chunk=f.read(1024*1024)
                if not chunk: break
                total+=len(chunk)
                if total>limit: raise ValueError(name+': size limit')
                digest.update(chunk)
        return digest.hexdigest()
    finally: os.close(fd)

def diagnose_orphans(db_path, bundle_root):
    """Inventory unregistered bundles and staging; no DB/file mutations.

    A valid manifest establishes internal consistency, not source provenance.
    Existing idempotency records are reported, never used to authorize recovery.
    """
    db_path=os.path.abspath(db_path)
    if os.path.exists(db_path+'-wal') and os.path.getsize(db_path+'-wal'):
        raise ValueError('checkpointed offline database snapshot required: nonempty WAL')
    uri = 'file:' + quote(db_path, safe='/') + '?mode=ro&immutable=1'
    con = sqlite3.connect(uri, uri=True)
    con.row_factory = sqlite3.Row
    try:
        con.execute('BEGIN')
        rows = [dict(r) for r in con.execute('SELECT map_version_id,storage_key,bundle_sha256,floor_id,source_mode FROM map_versions')]
        floor_ids = {r[0] for r in con.execute('SELECT floor_id FROM floors')}
        receipts = [dict(r) for r in con.execute('SELECT scope_key,response_json FROM map_write_idempotency')]
        by_key = {r['storage_key']: r for r in rows}
        by_id = {r['map_version_id']: r for r in rows}
        # Reject symlinked roots, not just symlinked child files.
        root_fd = os.open(os.path.abspath(bundle_root), os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
        try:
            bfd = os.open('bundles', os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=root_fd)
            try: sfd = os.open('.staging', os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=root_fd)
            except Exception:
                os.close(bfd)
                raise
            try:
                pending = [{'name': n, 'classification': 'STAGING_UNREVIEWED'} for n in sorted(os.listdir(sfd))]
                found = []
                for name in sorted(os.listdir(bfd)):
                    if name in by_key: continue
                    item = {'storage_key': name, 'classification': 'CORRUPT_OR_UNSAFE', 'recovery_allowed': False,
                            'source_verified': False, 'receipts': [], 'errors': []}
                    found.append(item)
                    fd = None
                    try:
                        fd = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=bfd)
                        for child in os.listdir(fd):
                            if stat.S_ISLNK(os.stat(child, dir_fd=fd, follow_symlinks=False).st_mode): raise ValueError(child+': symlink forbidden')
                        manifest = json.loads(_read_file(fd, 'manifest.json', MAX_YAML_BYTES))
                        if not isinstance(manifest, dict) or manifest.get('schema') != 'map_bundle_manifest/v1':
                            raise ValueError('invalid manifest schema')
                        files = {n: _read_file(fd,n,limit) for n,limit in [('map.yaml',MAX_YAML_BYTES),('map.pgm',MAX_PGM_BYTES)]}
                        hashes = manifest['sha256']
                        if not isinstance(hashes,dict): raise ValueError('invalid sha256 map')
                        pgsha = []
                        for n,k in [('map.posegraph','posegraph_sha256'),('map.posegraph.data','posegraph_data_sha256')]:
                            expected=hashes.get(k)
                            try: digest=_hash_file(fd,n,MAX_POSEGRAPH_BYTES)
                            except FileNotFoundError:
                                if expected is not None: raise ValueError(n+': missing')
                                digest=None
                            if digest != expected: raise ValueError(n+': checksum mismatch')
                            pgsha.append(digest)
                        if pgsha[1] and not pgsha[0]: raise ValueError('posegraph.data without posegraph')
                        ym=parse_ros_map_yaml(files['map.yaml']);pm=parse_pgm_p5(files['map.pgm'])
                        if hashes.get('yaml_sha256') != ym['yaml_sha256'] or hashes.get('pgm_sha256') != pm['pgm_sha256'] or hashes.get('content_sha256') != pm['content_sha256']:
                            raise ValueError('map checksum mismatch')
                        label=validate_entity_id(manifest['label'],'label');mid=make_map_version_id(label,pm['content_sha256'])
                        if manifest.get('map_version_id') != mid: raise ValueError('map version ID mismatch')
                        frame=validate_frame_id(manifest['frame_id'],field='frame_id')
                        floor=validate_entity_id(manifest['floor_id'],'floor_id')
                        if manifest.get('source_mode') not in SOURCE_MODES: raise ValueError('invalid source mode')
                        if ym['has_explicit_frame_id'] and ym['frame_id'] != frame: raise ValueError('frame mismatch')
                        bundle=compute_bundle_sha256(content_sha256=pm['content_sha256'],yaml_sha256=ym['yaml_sha256'],pgm_sha256=pm['pgm_sha256'],posegraph_sha256=pgsha[0],posegraph_data_sha256=pgsha[1],width=pm['width'],height=pm['height'],resolution=ym['resolution'],origin_x=ym['origin_x'],origin_y=ym['origin_y'],origin_yaw=ym['origin_yaw'],negate=ym['negate'],occupied_thresh=ym['occupied_thresh'],free_thresh=ym['free_thresh'],frame_id=frame)
                        expected_key=f"{label}__{pm['content_sha256'][:12]}__{bundle[:8]}"
                        if name != expected_key or hashes.get('bundle_sha256') != bundle: raise ValueError('storage key or bundle checksum mismatch')
                        if manifest.get('width') != pm['width'] or manifest.get('height') != pm['height'] or manifest.get('resolution') != ym['resolution'] or manifest.get('origin') != ym['origin']:
                            raise ValueError('manifest geometry mismatch')
                        existing=by_id.get(mid)
                        item.update(map_version_id=mid,bundle_sha256=bundle,
                                    receipts=[r['scope_key'] for r in receipts if mid in r['response_json']])
                        item['classification']=('DUPLICATE_REGISTERED' if existing and existing['bundle_sha256']==bundle and existing['floor_id']==floor and existing['source_mode']==manifest['source_mode'] else
                                                'ID_CONFLICT' if existing else 'INTACT_UNVERIFIED_ORIGIN')
                        item['floor_exists']=floor in floor_ids
                        item['reupload_expected']=('409_STORAGE_CONFLICT' if not existing else
                                                   '409_ID_CONFLICT' if item['classification']=='ID_CONFLICT' else
                                                   'INSPECT_REGISTERED_BUNDLE')
                    except (OSError, ValueError, TypeError, KeyError) as exc:
                        item['errors'].append(str(exc))
                    except Exception as exc:
                        # Registry parser errors are expected input failures; keep diagnostics non-mutating.
                        item['errors'].append(type(exc).__name__ + ': ' + str(exc))
                    finally:
                        if fd is not None: os.close(fd)
                return {'bundles':found,'staging':pending,'quiescent_snapshot_required':True}
            finally: os.close(bfd);os.close(sfd)
        finally: os.close(root_fd)
    finally: con.close()
