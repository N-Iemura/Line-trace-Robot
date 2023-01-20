#! /usr/bin/env python3

import rospy, cv_bridge	#ROSとOpenCV間でデータを受け渡すためのパッケージ
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		cv.namedWindow('BGR Image', 1)	#'BGR Image'という名前の画像表示のウィンドウを作成
		cv.namedWindow('MASK', 1)	#'MASK'という名前の画像表示のウィンドウを作成
		cv.namedWindow('MASKED', 1)	#'MASK'という名前の画像表示のウィンドウを作成
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)	#Image型で画像トピックを購読し，コールバック関数を呼ぶ
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

		self.twist = Twist()	#Twistインスタンス生成

	def image_callback(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')

		hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)	#色空間の変換(BGR→HSV)
		lower_yellow = np.array([10, 10, 10])		#黄色の閾値（下限）
		upper_yellow = np.array([255, 255, 250])	#黄色の閾値（上限）
		mask = cv.inRange(hsv, lower_yellow, upper_yellow)	#閾値によるHSV画像の2値化（マスク画像生成）
		masked = cv.bitwise_and(image, image, mask = mask)	#mask画像において，1である部分だけが残る（フィルタに通している）
		h, w = image.shape[:2]
		RESIZE = (w//3, h//3)
		search_top = (h//4)*3
		search_bot = search_top + 20	#目の前の線にだけに興味がある→20行分くらいに絞る
		mask[0:search_top, 0:w] = 0
		mask[search_bot:h, 0:w] = 0



		M = cv.moments(mask)	#maskにおける1の部分の重心
		if M['m00'] > 0:	#重心が存在する
			cx = int(M['m10']/M['m00'])	#重心のx座標
			cy = int(M['m01']/M['m00'])	#重心のy座標
			cv.circle(image, (cx, cy), 20, (0, 0, 255), -1)	#赤丸を画像に描画

			##P制御
			err = cx - w//2	#黄色の先の重心座標(x)と画像の中心(x)との差
			self.twist.linear.x = 0.2
			#self.twist.angular.z = -float(err)/100	#画像が大きいためか，-1/100では絶対値がまだ十分に大きく，ロボットが暴れてしまう
			self.twist.angular.z = -float(err)/100	#誤差にあわせて回転速度を変化させる（-1/1000がP制御でいうところの比例ゲインにあたる）
			self.cmd_vel_pub.publish(self.twist)

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

