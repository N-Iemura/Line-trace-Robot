#! /usr/bin/env python3

import rospy, cv_bridge	#ROSとOpenCV間でデータを受け渡すためのパッケージ
from sensor_msgs.msg import Image
import cv2 as cv
import numpy as np

class Follower:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		cv.namedWindow('BGR Image', 1)	#'BGR Image'という名前の画像表示のウィンドウを作成
		cv.namedWindow('MASK', 1)	#'MASK'という名前の画像表示のウィンドウを作成
		cv.namedWindow('MASKED', 1)	#'MASK'という名前の画像表示のウィンドウを作成
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)	#Image型で画像トピックを購読し，コールバック関数を呼ぶ

	def image_callback(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
		h, w = image.shape[:2]
		RESIZE = (w//3, h//3)
		hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)	#色空間の変換(BGR→HSV)
		lower_yellow = np.array([10, 10, 10])		#黄色の閾値（下限）
		upper_yellow = np.array([255, 255, 250])	#黄色の閾値（上限）
		mask = cv.inRange(hsv, lower_yellow, upper_yellow)	#閾値によるHSV画像の2値化（マスク画像生成）
		masked = cv.bitwise_and(image, image, mask = mask)	#mask画像において，1である部分だけが残る（フィルタに通している）

		#大きすぎるため，サイズ調整
		display_mask = cv.resize(mask, RESIZE)
		display_masked = cv.resize(masked, RESIZE)
		display_image = cv.resize(image, RESIZE)
		
		#表示
		cv.imshow('BGR Image', display_image)	#'BGR Image'ウィンドウにimageを表示
		cv.imshow('MASK', display_mask)			#'MASK'ウィンドウにimageを表示
		cv.imshow('MASKED', display_masked)		#'MASKED'ウィンドウにimageを表示
		cv.waitKey(3)	#3ミリ秒待つ

rospy.init_node('follower')	#'follower'という名前でノードを初期化
follower = Follower()	#Followerクラスのインスタンスを作成（init関数が実行される）
rospy.spin()	#ループ
