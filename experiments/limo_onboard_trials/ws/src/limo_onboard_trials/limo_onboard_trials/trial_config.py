"""시험 조건을 분리한다. 기존 USB 파일은 읽기 기준이며 덮어쓰지 않는다."""
from pathlib import Path
import math
import yaml

PROFILES = ('usb', 'bt', 'rpp_bt')


def configure(share, profile, backup, collision_time=''):
    share = Path(share)
    if profile not in PROFILES:
        raise ValueError('profile must be usb, bt, or rpp_bt')
    if backup not in ('15', '30'):
        raise ValueError('backup must be 15 or 30 (centimeters)')
    if profile == 'usb' and backup != '15':
        raise ValueError('USB baseline uses its default BT; backup override needs bt/rpp_bt')
    config = share / 'config' / {'usb': 'nav2_usb.yaml', 'bt': 'nav2_bt.yaml',
                                'rpp_bt': 'nav2_rpp.yaml'}[profile]
    rewrites = {}
    if profile != 'usb':
        # Humble은 없는 키를 추가하지 않으므로 시험용 YAML에 BT 키를 미리 명시했다.
        # 실행 시 설치 경로를 사용해 노트북의 절대 경로를 넣지 않는다.
        rewrites['default_nav_to_pose_bt_xml'] = str(share/'config'/f'recovery_backup{backup}.xml')
    if collision_time:
        # 충돌 예측 시간을 줄이는 것은 안전 강화와 같지 않다. 명시한 시험에서만 바꾼다.
        if profile != 'rpp_bt':
            raise ValueError('collision_time is only supported for rpp_bt')
        value = float(collision_time)
        if not math.isfinite(value) or value <= 0:
            raise ValueError('collision_time must be positive and finite')
        rewrites['max_allowed_time_to_collision_up_to_carrot'] = str(value)
    return str(config), rewrites


def validate_map(filename):
    path = Path(filename).expanduser().resolve()
    if not path.is_file():
        raise ValueError('Map YAML does not exist: ' + str(path))
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict) or not isinstance(document.get('image'), str):
        raise ValueError('Map YAML must contain an image filename')
    image = Path(document['image']).expanduser()
    if not image.is_absolute():
        image = path.parent/image
    if not image.is_file():
        raise ValueError('Map image does not exist: ' + str(image))
    return str(path)
