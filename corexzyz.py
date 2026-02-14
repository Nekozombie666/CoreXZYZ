# klippy/kinematics/corexzyz.py

import math
from . import stepper

class CoreXZYZKinematics:
    def __init__(self, toolhead, config):
        self.toolhead = toolhead
        
        # printer.cfg から各ステッパーの設定を読み込む
        # [stepper_a], [stepper_b], [stepper_c], [stepper_d] を定義することを想定
        self.rails = [
            stepper.PrinterRail(config.getsection('stepper_a')),
            stepper.PrinterRail(config.getsection('stepper_b')),
            stepper.PrinterRail(config.getsection('stepper_c')),
            stepper.PrinterRail(config.getsection('stepper_d')),
        ]
        
        # ステッパーに名前を割り当て（デバッグ用など）
        self.rails[0].setup_pin('step_pin')
        self.rails[1].setup_pin('step_pin')
        self.rails[2].setup_pin('step_pin')
        self.rails[3].setup_pin('step_pin')

    def get_steppers(self):
        # ツールヘッドに「このキネマティクスはこのモーターを使うよ」と伝える
        return [s.get_stepper() for s in self.rails]

    def calc_position(self, stepper_positions):
        # フォワードキネマティクス: モーターの位置からヘッドの座標(XYZ)を逆算する
        # stepper_positionsは {stepper_name: position} の辞書
        
        pos_a = stepper_positions.get(self.rails[0].get_name(), 0.)
        pos_b = stepper_positions.get(self.rails[1].get_name(), 0.)
        pos_c = stepper_positions.get(self.rails[2].get_name(), 0.)
        pos_d = stepper_positions.get(self.rails[3].get_name(), 0.)

        # 数式モデル:
        # A = Z + X, B = Z - X  =>  2Z = A+B, 2X = A-B
        # C = Z + Y, D = Z - Y  =>  2Z = C+D, 2Y = C-D
        
        x = 0.5 * (pos_a - pos_b)
        y = 0.5 * (pos_c - pos_d)
        
        # ZはXZ面とYZ面の平均を取る（剛性が高ければ同じ値になるはず）
        z_xz = 0.5 * (pos_a + pos_b)
        z_yz = 0.5 * (pos_c + pos_d)
        z = 0.5 * (z_xz + z_yz)

        return [x, y, z]

    def set_position(self, newpos, homing_axes):
        # 位置を強制的にセットする（G92やホーミング直後など）
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)

    def check_move(self, move):
        # インバースキネマティクス: 動きたい座標(XYZ)から各モーターのステップ位置を計算する
        # ここが最も重要です
        
        pos = move.axes_d
        x = pos[0]
        y = pos[1]
        z = pos[2]

        # ステップ位置の計算
        # X軸関連 (CoreXZ)
        m_a = z + x
        m_b = z - x
        
        # Y軸関連 (CoreYZ)
        m_c = z + y
        m_d = z - y

        # Klipperのモーションキューに登録
        move.move_d(self.rails[0], m_a)
        move.move_d(self.rails[1], m_b)
        move.move_d(self.rails[2], m_c)
        move.move_d(self.rails[3], m_d)

    def home(self, homing_state):
        # ホーミング処理
        # 通常は homing.py のヘルパーを使いますが、Core系は軸の干渉があるため
        # どの順番でエンドストップに当てるか定義が必要です。
        # ここでは簡易的に全軸ホームのロジックを呼び出す形にします。
        
        # Klipperの標準的なホーミングルーチンへ委譲（必要に応じてカスタマイズ）
        return self._home_axis(homing_state)

# Klipperがこのファイルを読み込んだ時にクラスをロードするためのフック
def load_kinematics(toolhead, config):
    return CoreXZYZKinematics(toolhead, config)
