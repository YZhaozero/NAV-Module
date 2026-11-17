"""WTRTK protocol catalog shared by the UI and serial owner (no I/O)."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Command:
    key: str
    group: str
    title: str
    wire: str
    reply: str
    choices: tuple = ()
    warning: str = ''
    unavailable: str = ''
    query: str = ''
    restart: bool = False


BAUDS = tuple((str(speed), str(code)) for code, speed in enumerate(
    (9600, 19200, 38400, 57600, 115200, 230400, 460800), 2))
# The document's 0x0F wire representation is not established; don't guess it.
RATES = tuple((f'{hz} Hz', str(code)) for hz, code in ((1, 3), (2, 4), (5, 5), (10, 6), (20, 7)))


def catalog():
    rows = [
        Command('AT', '连接与配置模式', '连接测试', 'AT', 'OK'),
        Command('CFG', '连接与配置模式', '进入配置模式', 'AT+CFG', '+CFG'),
        Command('EXIT', '连接与配置模式', '退出配置模式', 'AT+EXIT', '+EXIT'),
        Command('VERSION?', '设备信息与串口', '查询固件版本', 'AT+VERSION?', '+VERSION='),
        Command('BAUD?', '设备信息与串口', '查询波特率', 'AT+BAUD?', '+BAUD='),
        Command('BAUD', '设备信息与串口', '设置波特率', 'AT+BAUD={value}', '+BAUD=', BAUDS,
                '将切换接收器和本机串口速率；若未确认，请按设备实际速率重新连接。', query='BAUD?'),
        Command('MSG?', '报文输出', '查询输出报文', 'AT+MSG?', '+MSG='),
        Command('MSG', '报文输出', '设置输出报文', 'AT+MSG={value}', '+MSG=',
                unavailable='协议未完整定义报文选项及空位规则，暂不提供猜测参数。'),
    ]
    for key, title in (('IMU', 'WTRAW'), ('GGA', 'GGA'), ('RMC', 'RMC'), ('RTK', 'WTRTK')):
        rows.extend((Command(key+'?', '报文输出', f'查询 {title} 输出频率', f'AT+{key}?', f'+{key}='),
                     Command(key, '报文输出', f'设置 {title} 输出频率', f'AT+{key}={{value}}', f'+{key}=',
                             RATES, '将改变报文输出频率；不会自动保存。', query=key+'?'),
                     Command(key+'_OFF', '报文输出', f'关闭 {title} 输出', '', '',
                             unavailable='协议标记为0x0F，实际发送编码尚未验证，暂不可用。')))
    for key, title, choices in (
        ('RTKMODE', 'RTK工作模式', (('流动站', '0'), ('固定基站', '1'))),
        ('GPSMODE', '定位场景', (('车载', '0'), ('精准测量', '1'), ('无人机', '2'))),
    ):
        rows.extend((Command(key+'?', '工作模式', '查询'+title, f'AT+{key}?', f'+{key}='),
                     Command(key, '工作模式', '设置'+title, f'AT+{key}={{value}}', f'+{key}=', choices,
                             '设备将自动保存并重启；重启完成和恢复收报需另行确认。', restart=True)))
    rows.extend((
        Command('CAL', '校准与对准', '加速度计校准', 'AT+CAL', '+CAL',
                warning='请按设备校准要求放置并保持静止；回执不代表校准精度通过。'),
        Command('RTKCAL', '校准与对准', 'RTK安装对准', 'AT+RTKCAL', '+RTKCAL',
                warning='请先满足厂家安装对准条件；回执不代表已完成对准。'),
        Command('OPEN', '差分与参数维护', '允许接收RTCM', 'AT+OPEN ROVER', 'ROVER OPEN OK',
                warning='仅打开接收器RTCM入口，不连接差分服务器；调试期间差分注入暂停。'),
        Command('CLOSE', '差分与参数维护', '关闭RTCM入口', 'AT+CLOSE ROVER', 'ROVER CLOSE OK',
                warning='将停止接收器差分输入；调试期间差分自动控制暂停。'),
        Command('SAVE', '差分与参数维护', '保存当前配置', 'AT+SAVE', 'SAVE',
                warning='将当前配置写入设备，重新上电仍保留。'),
        Command('RST', '差分与参数维护', '恢复出厂并重启（RST）', 'AT+RST', 'RST',
                warning='协议将此指令定义为恢复出厂并重启，不是普通重启；串口速率等参数可能改变。', restart=True),
        Command('RESTORE', '差分与参数维护', '恢复出厂并重启（RESTORE）', 'AT+RESTORE', '+RESTORE',
                warning='将丢失设备配置并重启；出厂波特率需按具体设备确认。', restart=True),
    ))
    return tuple(rows)


COMMANDS = catalog()
BY_KEY = {row.key: row for row in COMMANDS}


def prepare(key, value='', confirmed=False):
    if not isinstance(key, str) or key not in BY_KEY or not isinstance(value, str):
        raise ValueError('不支持的指令')
    row = BY_KEY[key]
    if row.unavailable:
        raise ValueError(row.unavailable)
    if row.warning and confirmed is not True:
        raise ValueError('此操作需要确认')
    if row.choices and value not in dict((v, t) for t, v in row.choices):
        raise ValueError('参数不在协议允许范围内')
    if not row.choices and value:
        raise ValueError('此指令不接受参数')
    return row, row.wire.format(value=value).encode('ascii') + b'\r\n'


def custom_command(text, encoding='text', crlf=True, confirmed=False):
    """Custom input cannot bypass known-command validation or combine commands."""
    if not isinstance(text, str) or len(text) > 2048:
        raise ValueError('自定义内容过长')
    try:
        raw = bytes.fromhex(text) if encoding == 'hex' else text.encode('ascii')
    except (ValueError, UnicodeError) as exc:
        raise ValueError('请输入有效ASCII文本或HEX字节') from exc
    if encoding not in ('text', 'hex'):
        raise ValueError('不支持的编码')
    body = raw.removesuffix(b'\r\n')
    if not body or len(body) > 512 or b'\r' in body or b'\n' in body:
        raise ValueError('每次仅发送一条指令，最长512字节')
    for row in COMMANDS:
        prefix = row.wire.split('{value}')[0]
        if not prefix:
            continue
        if (row.choices or '{value}' in row.wire) and body.startswith(prefix.encode()):
            value = body[len(prefix):].decode('ascii')
            return (*prepare(row.key, value, confirmed), value)
        if body == row.wire.encode():
            return (*prepare(row.key, '', confirmed), '')
    if confirmed is not True:
        raise ValueError('自定义报文可能修改设备状态，需要确认')
    # Unknown commands remain unknown, never reported as acknowledged/successful.
    row = Command('CUSTOM', '自定义', '自定义发送', '', '', warning='结果需人工确认')
    return row, raw + (b'\r\n' if crlf and not raw.endswith(b'\r\n') else b''), ''
