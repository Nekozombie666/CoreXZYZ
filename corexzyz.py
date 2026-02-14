# corexzyz.py
import math
from . import stepper

class CoreXZYZKinematics:
    def __init__(self, toolhead, config):
        self.toolhead = toolhead
        
        # 4つのステッパーレールを定義します
        # 0, 1: XZ面を担当 (CoreXZ)
        # 2, 3: YZ面を担当 (CoreYZ)
        self.rails = [
            stepper.PrinterRail(config.getsection('stepper_a')),
            stepper.PrinterRail(config.getsection('stepper_b')),
            stepper.PrinterRail(config.getsection('stepper_c')),
            stepper.PrinterRail(config.getsection('stepper_d')),
        ]

        # ステッパーの初期化（ピン設定など）
        for rail in self.rails:
            rail.setup_pin('step_pin')
            rail.setup_pin('dir_pin')
            rail.setup_pin('enable_pin')

    def get_steppers(self):
        # ツールヘッドにこのキネマティクスが制御する全モーターを通知
        return [s.get_stepper() for s in self.rails]

    def calc_position(self, stepper_positions):
        # Forward Kinematics (モーター位置 -> XYZ座標)
        
        # 各モーターの現在位置を取得
        pos_a = stepper_positions.get(self.rails[0].get_name(), 0.)
        pos_b = stepper_positions.get(self.rails[1].get_name(), 0.)
        pos_c = stepper_positions.get(self.rails[2].get_name(), 0.)
        pos_d = stepper_positions.get(self.rails[3].get_name(), 0.)

        # CoreXZの計算 (A = Z + X, B = Z - X と仮定)
        # X = (A - B) / 2
        # Z1 = (A + B) / 2
        x = 0.5 * (pos_a - pos_b)
        z_xz = 0.5 * (pos_a + pos_b)

        # CoreYZの計算 (C = Z + Y, D = Z - Y と仮定)
        # Y = (C - D) / 2
        # Z2 = (C + D) / 2
        y = 0.5 * (pos_c - pos_d)
        z_yz = 0.5 * (pos_c + pos_d)

        # Z軸は2つの面の平均を取る（機械的な誤差を吸収するため）
        z = 0.5 * (z_xz + z_yz)

        return [x, y, z]

    def check_move(self, move):
        # Inverse Kinematics (XYZ移動命令 -> モーターステップ)
        
        # move.axes_d は [x, y, z, e] の目標座標
        x = move.axes_d[0]
        y = move.axes_d[1]
        z = move.axes_d[2]

        # CoreXZのロジック
        # モーターA: Z + X
        # モーターB: Z - X
        m_a = z + x
        m_b = z - x

        # CoreYZのロジック
        # モーターC: Z + Y
        # モーターD: Z - Y
        m_c = z + y
        m_d = z - y

        # 各モーターへ移動指示
        move.move_d(self.rails[0], m_a)
        move.move_d(self.rails[1], m_b)
        move.move_d(self.rails[2], m_c)
        move.move_d(self.rails[3], m_d)

    def set_position(self, newpos, homing_axes):
        # G92などで座標が強制変更された場合、全モーターの位置を再計算してセット
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)

    def home(self, homing_state):
        # ホーミング処理
        # CoreXZ/YZは軸が連動するため、単純なホーミングでは衝突する可能性があります。
        # 通常は [homing_override] を printer.cfg に書いてG-codeで制御するのが安全です。
        # ここでは最低限の軸登録のみ行います。
        return self._home_axis(homing_state)

def load_kinematics(toolhead, config):
    return CoreXZYZKinematics(toolhead, config)
