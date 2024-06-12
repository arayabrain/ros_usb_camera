import cv2
import numpy as np
import argparse


def undistort_image(image_path, camera_matrix, dist_coeffs):
    # 画像の読み込み
    img = cv2.imread(image_path)

    # 歪み補正
    h, w = img.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), 1, (w, h)
    )
    undistorted_img = cv2.undistort(
        img, camera_matrix, dist_coeffs, None, new_camera_matrix
    )

    # # ROIで画像をクロッピング
    # x, y, w, h = roi
    # undistorted_img = undistorted_img[y : y + h, x : x + w]

    return img, undistorted_img


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Undistort images using camera calibration data."
    )
    parser.add_argument(
        "calibration_data_path",
        type=str,
        help="Path to the camera calibration data file (.npz).",
    )
    parser.add_argument(
        "image_path", type=str, help="Path to the image to be undistorted."
    )
    return parser.parse_args()


def main():
    args = parse_arguments()

    # キャリブレーションデータの読み込み
    calib_data = np.load(args.calibration_data_path)
    mtx = calib_data["mtx"]
    dist = calib_data["dist"]

    # 画像の歪み補正
    input, output = undistort_image(args.image_path, mtx, dist)

    cv2.imshow("Comparison", output)

    # 画像の表示（オプション）
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
