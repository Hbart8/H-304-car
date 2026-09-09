# CanMV K230 自动避障小车 - 发挥部分第(2)项路线识别与串口发送
#
# 使用方法：
# 1. 在 CanMV IDE K230 中打开本文件，按现场相机安装角度调整“用户需要校准的参数”。
# 2. K230 上电后一键启动，程序会在准备区内识别 3x3 圆柱颜色排列。
# 3. 识别稳定后，按 500ms 周期向 MSPM0G3507 发送 1 个字节：
#       0x01 表示路线 1，0x02 表示路线 2，以此类推。
# 4. MSPM0G3507 端必须按本文件 ROUTE_LIBRARY 中的路线编号实现对应动作。
#
# 重要限制：
# - 本程序不使用任何神经网络模型，只使用颜色阈值、九宫格 ROI 和规则搜索。
# - 本文件同时包含本地自测代码；在 CanMV IDE 正常运行时不会执行自测。


# =========================
# 用户需要校准的参数
# =========================

# 摄像头输出分辨率。分辨率越高识别越细，但帧率越低。
DETECT_WIDTH = 640
DETECT_HEIGHT = 480

# 9 个检测框的位置，格式均为 (x, y, w, h)。
# 现在不再自动平均分九宫格，而是每个框单独调，适合固定相机的 3D 透视画面。
# 调法：
# - 先让小车/摄像头固定在比赛时的位置；
# - 运行程序后看右侧画面；
# - 逐个修改下面 9 个框，让框只压住圆柱主体，尽量避开地面线、阴影和强反光。
CELL_ROIS = [
    [(10, 0, 20, 70), (156, 168, 16, 135), (460, 170, 16, 140)],  # 第 1 行
    [(12, 180, 15, 90), (215, 180, 14, 100), (416, 175, 14, 105)],  # 第 2 行
    [(96, 190, 10, 75), (245, 190, 8, 60), (395, 186, 8, 65)],  # 第 3 行
]

# 每个框是否参与识别。False 表示该位置不识别，结果记为未知。
# 如果中间列距离短、互相干扰明显，可以保持下面默认值：只识别左右两列。
CELL_ENABLED = [
    [False, True, True],
    [True, True, True],
    [True, True, True],
]

# 亮度识别参数。
# 默认先使用动态分组：把当前启用 ROI 的亮度从小到大排序，
# 找最大亮度间隔，低亮度组判黑柱，高亮度组判白柱。
# 例如你截图中 22、29、32、34 与 50、52、53、56 之间有明显断层，
# 程序会自动把前一组判黑、后一组判白。
USE_DYNAMIC_LUMA_SPLIT = True
LUMA_CLUSTER_MIN_GAP = 8

# 如果动态分组失败，才使用下面两个固定阈值兜底。
# 这两个值按你当前截图先调成适合低曝光画面的范围。
BLACK_LUMA_MAX = 42
WHITE_LUMA_MIN = 48

# 至少识别出多少个有效黑/白框，才允许输出路线。
# 这样可以避免全是未知时直接走边路保底。
MIN_KNOWN_CELLS_FOR_ROUTE = 4

# 连续识别多少帧后进行多数投票。数值越大越稳，但启动时间越长。
VOTE_FRAME_COUNT = 18

# 最长探测时间。题目允许 30s 内完成探测并进入 A 口，这里留出余量给小车起步。
DETECT_TIMEOUT_MS = 26000

# 串口参数。K230 与 MSPM0G3507 必须共地，且 TX/RX 交叉连接。
# 默认使用 UART1：K230_TX -> MSPM0_RX，K230_RX <- MSPM0_TX。
UART_ID_NAME = "UART1"
UART_BAUDRATE = 115200
UART_TX_PIN = 3
UART_RX_PIN = 4

# 发送周期：题目要求 K230 计算好路线后每隔 500ms 发送一次路线字节。
SEND_PERIOD_MS = 500

# 识别失败时发送的字节。0x00 不代表任何路线，便于 MSPM0G3507 保持等待/停车。
ROUTE_UNKNOWN_BYTE = 0x00

# 调试开关。比赛前建议 True，正式测试可改 False 减少显示绘制开销。
SHOW_DEBUG_IMAGE = True
PRINT_DEBUG = True


# =========================
# 路线库定义
# =========================

# 将场地 3 行圆柱之间抽象为 4 条“横向通道”：
#   通道 1：第 1 行圆柱上方，靠上边墙
#   通道 2：第 1 行与第 2 行圆柱之间
#   通道 3：第 2 行与第 3 行圆柱之间
#   通道 4：第 3 行圆柱下方，靠下边墙
#
# 每条路线用 3 个通道编号表示：
#   第 1 个数：经过第 1 列圆柱时走哪条通道
#   第 2 个数：经过第 2 列圆柱时走哪条通道
#   第 3 个数：经过第 3 列圆柱时走哪条通道
#
# MSPM0G3507 端要按相同编号执行动作。例如：
#   route_id = 1，对应 ROUTE_LIBRARY[0]，即 (2, 2, 2)。
#
# 这些路线覆盖了“优先走中间、必要时短距离换道、最后才走边通道”的常用情况。
# 如果你的底盘动作库更多，可以继续追加路线；路线编号会自然递增。
ROUTE_LIBRARY = [
    (2, 2, 2),  # 1：中上通道直行，通常最短
    (3, 3, 3),  # 2：中下通道直行，通常最短
    (2, 2, 3),  # 3：前两列走中上，第三列切到中下
    (2, 3, 3),  # 4：第一列走中上，后两列走中下
    (3, 3, 2),  # 5：前两列走中下，第三列切到中上
    (3, 2, 2),  # 6：第一列走中下，后两列走中上
    (2, 1, 2),  # 7：中间一列借上边通道绕开
    (3, 4, 3),  # 8：中间一列借下边通道绕开
    (1, 1, 1),  # 9：全程上边通道，作为保底路线
    (4, 4, 4),  # 10：全程下边通道，作为保底路线
    (2, 1, 1),  # 11：后半程走上边通道
    (1, 1, 2),  # 12：前半程走上边通道
    (3, 4, 4),  # 13：后半程走下边通道
    (4, 4, 3),  # 14：前半程走下边通道
    (1, 2, 3),  # 15：从上方逐步回到中下
    (4, 3, 2),  # 16：从下方逐步回到中上
]


# =========================
# 纯规则路线规划代码
# =========================

COLOR_BLACK = "B"
COLOR_WHITE = "W"
COLOR_UNKNOWN = "?"


def _is_black(board, row, col):
    """判断 board[row][col] 是否为黑柱；row/col 均为 0 起始。"""
    return board[row][col] == COLOR_BLACK


def is_lane_blocked(board, lane, col):
    """判断某一列的某条通道是否被“两个黑柱之间穿过”规则禁止。

    参数：
    - board：3x3 字符矩阵，B=黑柱，W=白柱，?=未知。
    - lane：1~4 的通道编号。
    - col：0~2 的列编号。

    规则：
    - 通道 2 位于第 1 行和第 2 行之间，如果这两个位置都是黑柱，就不能走。
    - 通道 3 位于第 2 行和第 3 行之间，如果这两个位置都是黑柱，就不能走。
    - 通道 1/4 靠边墙，不属于两个圆柱之间，因此不按黑黑夹缝禁止。
    - 如果颜色未知，为了保守，认为可能有风险，禁止中间通道。
    """
    if lane == 2:
        top = board[0][col]
        bottom = board[1][col]
        if top == COLOR_UNKNOWN or bottom == COLOR_UNKNOWN:
            return True
        return top == COLOR_BLACK and bottom == COLOR_BLACK
    if lane == 3:
        top = board[1][col]
        bottom = board[2][col]
        if top == COLOR_UNKNOWN or bottom == COLOR_UNKNOWN:
            return True
        return top == COLOR_BLACK and bottom == COLOR_BLACK
    return False


def is_route_valid(board, route):
    """判断一条路线在 3 列上是否都不违反黑黑夹缝规则。"""
    for col in range(3):
        if is_lane_blocked(board, route[col], col):
            return False
    return True


def route_cost(route):
    """给路线计算代价，用于在多条可走路线中选更适合快速通过的一条。

    代价设计：
    - 中间通道 2/3 优先，因为离入口 A 和出口 C 更近。
    - 换道次数越少越好，减少底盘动作时间和误差。
    - 上/下边通道 1/4 作为保底，代价较高。
    """
    cost = 0

    # 使用边通道说明绕行幅度更大，给较高惩罚。
    for lane in route:
        if lane == 1 or lane == 4:
            cost += 6
        else:
            cost += 0

    # 每发生一次换道，增加代价。
    for i in range(1, len(route)):
        if route[i] != route[i - 1]:
            cost += 4

    # 起点/终点偏离中间通道也略增代价。
    cost += abs(route[0] - 2.5)
    cost += abs(route[-1] - 2.5)
    return cost


def choose_route_id(board):
    """根据 3x3 圆柱颜色矩阵选择路线编号。

    返回值：
    - 1~len(ROUTE_LIBRARY)：可执行路线编号。
    - 0：没有找到安全路线，或有效识别框太少，MSPM0G3507 应停车等待或报警。
    """
    known_count = 0
    for row in range(3):
        for col in range(3):
            if board[row][col] == COLOR_BLACK or board[row][col] == COLOR_WHITE:
                known_count += 1

    # 少量未知可以接受，但全未知或已知点太少时不能输出保底路线。
    if known_count < MIN_KNOWN_CELLS_FOR_ROUTE:
        return 0

    best_id = 0
    best_cost = None

    for index, route in enumerate(ROUTE_LIBRARY):
        if not is_route_valid(board, route):
            continue

        cost = route_cost(route)
        if best_cost is None or cost < best_cost:
            best_cost = cost
            best_id = index + 1

    return best_id


def empty_vote_table():
    """创建 3x3 投票表。每格分别统计 B/W/? 出现次数。"""
    table = []
    for _row in range(3):
        row = []
        for _col in range(3):
            row.append({COLOR_BLACK: 0, COLOR_WHITE: 0, COLOR_UNKNOWN: 0})
        table.append(row)
    return table


def board_from_votes(votes):
    """将多帧识别投票结果压缩成最终 3x3 棋盘。"""
    board = []
    for row in range(3):
        out_row = []
        for col in range(3):
            cell_votes = votes[row][col]
            black_count = cell_votes[COLOR_BLACK]
            white_count = cell_votes[COLOR_WHITE]
            unknown_count = cell_votes[COLOR_UNKNOWN]

            if black_count > white_count and black_count > unknown_count:
                out_row.append(COLOR_BLACK)
            elif white_count > black_count and white_count > unknown_count:
                out_row.append(COLOR_WHITE)
            else:
                out_row.append(COLOR_UNKNOWN)
        board.append(out_row)
    return board


def board_to_text(board):
    """把 3x3 棋盘转成便于串口/终端调试阅读的文本。"""
    lines = []
    for row in board:
        lines.append(" ".join(row))
    return "\n".join(lines)


def _self_test():
    """本地自测：只验证路线规划规则，不依赖 K230 摄像头库。

    在电脑上运行：
        python k230_route_planner.py --self-test
    """
    cases = [
        (
            "全白时应优先选择路线1：中上通道直行",
            [
                [COLOR_WHITE, COLOR_WHITE, COLOR_WHITE],
                [COLOR_WHITE, COLOR_WHITE, COLOR_WHITE],
                [COLOR_WHITE, COLOR_WHITE, COLOR_WHITE],
            ],
            1,
        ),
        (
            "第1列中上通道被黑黑夹住时，应避开路线1",
            [
                [COLOR_BLACK, COLOR_WHITE, COLOR_WHITE],
                [COLOR_BLACK, COLOR_WHITE, COLOR_WHITE],
                [COLOR_WHITE, COLOR_WHITE, COLOR_WHITE],
            ],
            2,
        ),
        (
            "中上和中下都局部受阻时，应选择可绕行路线",
            [
                [COLOR_BLACK, COLOR_WHITE, COLOR_WHITE],
                [COLOR_BLACK, COLOR_BLACK, COLOR_WHITE],
                [COLOR_WHITE, COLOR_BLACK, COLOR_WHITE],
            ],
            6,
        ),
        (
            "已知框太少时不能输出路线",
            [
                [COLOR_UNKNOWN, COLOR_WHITE, COLOR_WHITE],
                [COLOR_UNKNOWN, COLOR_UNKNOWN, COLOR_UNKNOWN],
                [COLOR_UNKNOWN, COLOR_UNKNOWN, COLOR_UNKNOWN],
            ],
            0,
        ),
        (
            "中间列未知但左右列已知时，可以选择中间列走边通道的路线",
            [
                [COLOR_WHITE, COLOR_UNKNOWN, COLOR_WHITE],
                [COLOR_WHITE, COLOR_UNKNOWN, COLOR_WHITE],
                [COLOR_WHITE, COLOR_UNKNOWN, COLOR_WHITE],
            ],
            7,
        ),
    ]

    for name, board, expected in cases:
        actual = choose_route_id(board)
        assert actual == expected, "%s：期望 %d，实际 %d\n%s" % (
            name,
            expected,
            actual,
            board_to_text(board),
        )

    assert classify_brightness(35) == COLOR_BLACK
    assert classify_brightness(55) == COLOR_WHITE
    assert classify_brightness(45) == COLOR_UNKNOWN

    screenshot_scores = [
        [-1, 32, 53],
        [22, 34, 56],
        [50, 29, 52],
    ]
    screenshot_board = classify_board_by_luma(screenshot_scores)
    assert screenshot_board == [
        [COLOR_UNKNOWN, COLOR_BLACK, COLOR_WHITE],
        [COLOR_BLACK, COLOR_BLACK, COLOR_WHITE],
        [COLOR_WHITE, COLOR_BLACK, COLOR_WHITE],
    ], board_to_text(screenshot_board)

    print("self-test passed: %d cases" % len(cases))


# =========================
# K230 图像识别代码
# =========================


def calc_cell_roi(row, col):
    """返回第 row 行、第 col 列的自定义检测框。"""
    return CELL_ROIS[row][col]


def is_cell_enabled(row, col):
    """判断某个检测框是否启用。"""
    return CELL_ENABLED[row][col]


def classify_brightness(luma):
    """根据亮度值判断黑柱/白柱/未知。

    亮度范围通常为 0~255：
    - 黑柱吸光，亮度低；
    - 白柱反光，亮度高；
    - 中间区域可能是阴影、地面、边线或过曝混合，保守记为未知。
    """
    if luma <= BLACK_LUMA_MAX:
        return COLOR_BLACK
    if luma >= WHITE_LUMA_MIN:
        return COLOR_WHITE
    return COLOR_UNKNOWN


def classify_board_by_luma(scores):
    """根据 3x3 亮度表生成黑/白/未知棋盘。

    优先使用动态分组：
    - 只统计启用且亮度有效的 ROI；
    - 按亮度排序，寻找相邻亮度的最大间隔；
    - 最大间隔足够大时，间隔左侧判黑柱，右侧判白柱。

    这样可以适配不同曝光：即使白柱亮度只有 50 多，只要它明显比黑柱亮，
    也不会被固定阈值误判成黑柱。
    """
    board = []
    valid_values = []

    for row in range(3):
        board_row = []
        for col in range(3):
            luma = scores[row][col]
            if is_cell_enabled(row, col) and luma >= 0:
                valid_values.append(luma)
            board_row.append(COLOR_UNKNOWN)
        board.append(board_row)

    split_threshold = None
    if USE_DYNAMIC_LUMA_SPLIT and len(valid_values) >= 2:
        sorted_values = sorted(valid_values)
        best_gap = -1
        best_index = -1
        for index in range(len(sorted_values) - 1):
            gap = sorted_values[index + 1] - sorted_values[index]
            if gap > best_gap:
                best_gap = gap
                best_index = index

        if best_gap >= LUMA_CLUSTER_MIN_GAP:
            split_threshold = (sorted_values[best_index] + sorted_values[best_index + 1]) / 2.0

    for row in range(3):
        for col in range(3):
            luma = scores[row][col]
            if not is_cell_enabled(row, col) or luma < 0:
                board[row][col] = COLOR_UNKNOWN
            elif split_threshold is not None:
                if luma <= split_threshold:
                    board[row][col] = COLOR_BLACK
                else:
                    board[row][col] = COLOR_WHITE
            else:
                board[row][col] = classify_brightness(luma)

    return board


def get_roi_luma(img, roi):
    """读取 ROI 的平均亮度。

    优先使用 CanMV/OpenMV 的 get_statistics(roi=...)。
    如果某个固件接口不同，则退回到 ROI 中心点采样，保证程序不直接停止。
    """
    try:
        stats = img.get_statistics(roi=roi)
        try:
            return int(stats.l_mean())
        except Exception:
            return int(stats.mean())
    except Exception:
        pass

    # 兜底：采样 ROI 中心点 RGB，再换算近似亮度。
    try:
        x = roi[0] + roi[2] // 2
        y = roi[1] + roi[3] // 2
        pixel = img.get_pixel(x, y)
        if isinstance(pixel, tuple) and len(pixel) >= 3:
            return int(pixel[0] * 0.299 + pixel[1] * 0.587 + pixel[2] * 0.114)
        return int(pixel)
    except Exception:
        return -1


def classify_cell(img, roi, enabled=True):
    """识别单个圆柱格子的颜色。

    分类思路：
    - 如果该框禁用，直接返回未知；
    - 如果启用，计算 ROI 平均亮度；
    - 亮度低判黑柱，亮度高判白柱，中间亮度判未知。
    """
    if not enabled:
        return COLOR_UNKNOWN, -1, 0

    luma = get_roi_luma(img, roi)
    if luma < 0:
        return COLOR_UNKNOWN, luma, 0

    return classify_brightness(luma), luma, 0


def detect_board_once(img):
    """对当前图像识别一次 3x3 圆柱颜色。"""
    scores = []
    for row in range(3):
        score_row = []
        for col in range(3):
            roi = calc_cell_roi(row, col)
            _color, luma, _unused = classify_cell(img, roi, is_cell_enabled(row, col))
            score_row.append(luma)
        scores.append(score_row)

    board = classify_board_by_luma(scores)
    return board, scores


def update_votes(votes, board):
    """把单帧识别结果加入多帧投票表。"""
    for row in range(3):
        for col in range(3):
            color = board[row][col]
            if color != COLOR_BLACK and color != COLOR_WHITE:
                color = COLOR_UNKNOWN
            votes[row][col][color] += 1


def draw_debug(img, board, route_id, scores=None):
    """在 IDE 图像上绘制 9 个 ROI、识别结果和路线编号，便于现场调参。"""
    # 不同识别结果使用不同颜色画框。
    color_map = {
        COLOR_BLACK: (255, 0, 0),      # 红框：识别为黑柱
        COLOR_WHITE: (0, 255, 0),      # 绿框：识别为白柱
        COLOR_UNKNOWN: (255, 255, 0),  # 黄框：未知
    }

    for row in range(3):
        for col in range(3):
            roi = calc_cell_roi(row, col)
            if is_cell_enabled(row, col):
                color = color_map.get(board[row][col], (255, 255, 0))
            else:
                color = (120, 120, 120)
            img.draw_rectangle(roi, color=color, thickness=2)
            label = board[row][col]
            if scores is not None:
                luma = scores[row][col]
                if luma >= 0:
                    label = "%s %d" % (label, luma)
                else:
                    label = "%s off" % label
            img.draw_string_advanced(roi[0] + 3, roi[1] + 3, 16, label, color=color)

    img.draw_string_advanced(8, 8, 18, "route: %d" % route_id, color=(0, 255, 255))


def get_uart_class_id(UART):
    """把 UART_ID_NAME 转换成 CanMV UART 类常量。"""
    if UART_ID_NAME == "UART1":
        return UART.UART1
    if UART_ID_NAME == "UART2":
        return UART.UART2
    if UART_ID_NAME == "UART4":
        return UART.UART4
    return UART.UART1


def setup_uart():
    """初始化 K230 UART，并返回串口对象。"""
    from machine import FPIOA
    from machine import UART

    fpioa = FPIOA()

    # K230 的物理管脚需要先通过 FPIOA 复用成 UART 功能。
    # 如果你的开发板 UART1_TX/RX 不是 GPIO3/GPIO4，只改上面的 UART_TX_PIN/UART_RX_PIN。
    if UART_ID_NAME == "UART1":
        fpioa.set_function(UART_TX_PIN, FPIOA.UART1_TXD)
        fpioa.set_function(UART_RX_PIN, FPIOA.UART1_RXD)
    elif UART_ID_NAME == "UART2":
        fpioa.set_function(UART_TX_PIN, FPIOA.UART2_TXD)
        fpioa.set_function(UART_RX_PIN, FPIOA.UART2_RXD)
    elif UART_ID_NAME == "UART4":
        fpioa.set_function(UART_TX_PIN, FPIOA.UART4_TXD)
        fpioa.set_function(UART_RX_PIN, FPIOA.UART4_RXD)

    uart = UART(
        get_uart_class_id(UART),
        baudrate=UART_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )
    return uart


def setup_camera_and_display():
    """初始化 K230 摄像头、IDE 虚拟显示和媒体管理器。"""
    from media.sensor import Sensor
    from media.display import Display
    from media.media import MediaManager

    sensor = Sensor(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.reset()

    # 如果图像左右/上下与实际场地相反，可打开下面两行之一。
    # sensor.set_hmirror(True)
    # sensor.set_vflip(True)

    sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)

    # VIRT 表示把图像送回 CanMV IDE，to_ide=True 是 K230 IDE 预览的关键参数。
    Display.init(Display.VIRT, width=DETECT_WIDTH, height=DETECT_HEIGHT, fps=30, to_ide=True)
    MediaManager.init()
    sensor.run()
    return sensor, Display, MediaManager


def should_exit(os_module):
    """兼容 CanMV 的停止检查；普通异常不影响主循环。"""
    try:
        os_module.exitpoint()
    except Exception:
        pass


def detect_until_stable(sensor, Display, os_module, time_module):
    """在探测时间内进行多帧投票，返回最终路线编号和棋盘。

    这里不会等满 26s：达到 VOTE_FRAME_COUNT 帧后就会给出结果，
    这样 MSPM0G3507 可以更早收到路线并启动。
    """
    start_ms = time_module.ticks_ms()
    votes = empty_vote_table()
    frame_count = 0
    last_board = [
        [COLOR_UNKNOWN, COLOR_UNKNOWN, COLOR_UNKNOWN],
        [COLOR_UNKNOWN, COLOR_UNKNOWN, COLOR_UNKNOWN],
        [COLOR_UNKNOWN, COLOR_UNKNOWN, COLOR_UNKNOWN],
    ]
    route_id = 0
    last_img = None

    while time_module.ticks_diff(time_module.ticks_ms(), start_ms) < DETECT_TIMEOUT_MS:
        should_exit(os_module)
        img = sensor.snapshot()
        last_img = img
        board, scores = detect_board_once(img)
        update_votes(votes, board)
        frame_count += 1

        voted_board = board_from_votes(votes)
        route_id = choose_route_id(voted_board)
        last_board = voted_board

        if SHOW_DEBUG_IMAGE:
            draw_debug(img, voted_board, route_id, scores)
            Display.show_image(img)

        if PRINT_DEBUG:
            print("frame:", frame_count, "route:", route_id)
            print(board_to_text(voted_board))

        # 达到足够投票帧数且路线有效，就提前结束探测。
        if frame_count >= VOTE_FRAME_COUNT and route_id > 0:
            return route_id, last_board, last_img

    # 超时后仍返回当前最佳结果；若 route_id 为 0，主循环会发送 0x00。
    return route_id, last_board, last_img


def send_route_forever(uart, route_id, board, last_img, Display, sensor, time_module, os_module):
    """按 500ms 周期重复发送路线编号，同时持续刷新 IDE 图像。

    CanMV IDE 右侧预览依赖 Display.show_image() 持续送图。
    如果路线算完后只发串口、不再送图，IDE 可能显示 FPS=0 或“没有图像”。
    """
    if route_id <= 0:
        tx_byte = ROUTE_UNKNOWN_BYTE
    else:
        tx_byte = route_id & 0xFF

    payload = bytes([tx_byte])
    next_send_ms = time_module.ticks_ms()
    next_show_ms = time_module.ticks_ms()

    while True:
        should_exit(os_module)
        now = time_module.ticks_ms()
        if time_module.ticks_diff(now, next_send_ms) >= 0:
            if uart is not None:
                uart.write(payload)
            if PRINT_DEBUG:
                print("uart send: 0x%02X" % tx_byte)
            next_send_ms = time_module.ticks_add(now, SEND_PERIOD_MS)

        # 每 50ms 刷一次调试图像。路线确定后仍继续 snapshot，方便现场观察。
        if SHOW_DEBUG_IMAGE and time_module.ticks_diff(now, next_show_ms) >= 0:
            try:
                img = sensor.snapshot()
            except Exception:
                img = last_img
            if img is not None:
                current_board, current_scores = detect_board_once(img)
                draw_debug(img, current_board, route_id, current_scores)
                Display.show_image(img)
            next_show_ms = time_module.ticks_add(now, 50)
        time_module.sleep_ms(10)


def run_on_k230():
    """K230 主程序入口。"""
    import os
    import time

    sensor = None
    Display = None
    MediaManager = None
    uart = None

    try:
        # 官方示例在主入口先启用 exitpoint，IDE 停止按钮和资源释放会更稳定。
        try:
            os.exitpoint(os.EXITPOINT_ENABLE)
        except Exception:
            pass

        sensor, Display, MediaManager = setup_camera_and_display()

        # 先启动摄像头和 IDE 预览，再初始化 UART。
        # 这样即使串口引脚配置错误，也不会表现成“没有图像”。
        try:
            uart = setup_uart()
        except BaseException as err:
            uart = None
            print("uart init failed:", err)

        route_id, board, last_img = detect_until_stable(sensor, Display, os, time)

        if PRINT_DEBUG:
            print("final route:", route_id)
            print(board_to_text(board))

        send_route_forever(uart, route_id, board, last_img, Display, sensor, time, os)

    except KeyboardInterrupt:
        print("user stop")
    except BaseException as err:
        print("exception:", err)
    finally:
        # 释放资源，避免 CanMV IDE 下次运行时摄像头/显示占用。
        try:
            if sensor is not None:
                sensor.stop()
        except Exception:
            pass

        try:
            if Display is not None:
                Display.deinit()
        except Exception:
            pass

        try:
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
            time.sleep_ms(100)
        except Exception:
            pass

        try:
            if MediaManager is not None:
                MediaManager.deinit()
        except Exception:
            pass

        try:
            if uart is not None:
                uart.deinit()
        except Exception:
            pass


if __name__ == "__main__":
    # 普通电脑上只用于路线规划自测；CanMV IDE 正常运行时没有 --self-test 参数，
    # 因而会进入 run_on_k230()。
    try:
        import sys
        if "--self-test" in sys.argv:
            _self_test()
        else:
            run_on_k230()
    except ImportError:
        # 某些 MicroPython 固件没有 sys 模块时，直接进入 K230 主程序。
        run_on_k230()
