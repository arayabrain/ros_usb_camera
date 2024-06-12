import numpy as np
import cv2
import glob
import argparse


def calibrate_camera(args):
    # チェスボードの設定
    chessboard_size = (10, 7)  # チェスボードのコーナーの数（内角）
    square_size = 21.6  # 正方形のサイズ（mmなど、実際の測定単位）

    # 3Dポイントの準備（実世界の座標系）
    objp = np.zeros((np.prod(chessboard_size), 3), np.float32)
    objp[:, :2] = np.indices(chessboard_size).T.reshape(-1, 2) * square_size

    # 画像点とオブジェクト点の格納用リスト
    objpoints = []  # 3Dの点（実世界）
    imgpoints = []  # 2Dの点（画像上）

    # 画像の読み込み
    images = glob.glob(f"{args.image_dir}/*.png")  # 画像が保存されているパス
    print(f"{len(images)}枚の画像を読み込みます。")

    for idx, fname in enumerate(images):
        if idx % 10 != 0:
            continue
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # チェスボードのコーナーを検出
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

        # コーナーが見つかった場合、データに追加
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                criteria=(
                    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    30,
                    0.001,
                ),
            )
            imgpoints.append(corners2)

            # 結果の表示（オプション）
            cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
            cv2.imshow("img", img)
            cv2.waitKey(500)

    cv2.destroyAllWindows()

    # カメラのキャリブレーションを実行
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    # キャリブレーションの結果を表示
    print("Camera matrix:\n", mtx)
    print("Distortion coefficients:\n", dist)
    print("Rotation Vectors:\n", rvecs)
    print("Translation Vectors:\n", tvecs)

    # キャリブレーションデータの保存
    np.savez("calibration_data.npz", mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Camera calibration with chessboard images."
    )
    parser.add_argument(
        "image_dir",
        type=str,
        help="Directory containing chessboard images for calibration.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_arguments()
    calibrate_camera(args)
