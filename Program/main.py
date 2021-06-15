import cv2
from RRT import RRT

if __name__ == "__main__":
    img = cv2.imread('housemap.bmp')
    solver = RRT(img, (50, 700), (850, 50), (1080, 777))  # (50, 700), (850, 50)
    solver.run()
    cv2.imshow("img", solver.mmap)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
