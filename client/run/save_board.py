import cv2 as cv
import cv2.aruco as aruco

# from boarddef import ARUCO_TEST_BOARD_DEFINITION, ARUCO_TEST_BOARD_IDS

# board = aruco.Board_create(
#     ARUCO_TEST_BOARD_DEFINITION, aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL), ARUCO_TEST_BOARD_IDS)
# img = board.draw((1920, 1080))
# cv.imwrite("test-board.png", img)


from ..config import FINAL_BOARD
img = FINAL_BOARD.draw((500, 400))
cv.imwrite("final-board.png", img)
