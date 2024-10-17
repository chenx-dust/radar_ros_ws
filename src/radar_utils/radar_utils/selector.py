import os
from ament_index_python import get_package_share_directory

import cv2

configs = {
    "real_width": 28.0,
    "real_height": 15.0,
}

down = False

def draw(x, y):
    global img, ori_img
    print(f'({x / ori_img.shape[1] * configs["real_width"]:.3f}, {(1 - y / ori_img.shape[0]) * configs["real_height"]:.3f})')
    font = cv2.FONT_HERSHEY_SIMPLEX
    img = ori_img.copy()
    cv2.circle(img, (x, y), 1, (255, 255, 255), -1)
    cv2.putText(img, f'({x / ori_img.shape[1] * configs["real_width"]:.3f}, {(1 - y / ori_img.shape[0]) * configs["real_height"]:.3f})', (x, y), font,
                0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imshow('Image', img)

def show_coordinates(event, x, y, flags, param):
    global img, ori_img, down
    if event == cv2.EVENT_LBUTTONDOWN:
        down = True
        draw(x, y)
    elif event == cv2.EVENT_LBUTTONUP:
        down = False
        draw(x, y)
    elif event == cv2.EVENT_MOUSEMOVE and down:
        draw(x, y)

def main():
    global img, ori_img
    ori_img = cv2.imread(os.path.join(
        get_package_share_directory('radar_bringup'), 'resource', 'map.png'))
    img = ori_img.copy()
    cv2.namedWindow('Image')
    cv2.setMouseCallback('Image', show_coordinates)
    while True:
        cv2.imshow('Image', img)
        # 按下 'q' 键退出循环
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
