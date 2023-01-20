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
		cv.namedWindow('MASK_yellow', 1)	
		cv.namedWindow('MASKED_yellow', 1)	
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)	#Image型で画像トピックを購読し，コールバック関数を呼ぶ
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

		self.twist = Twist()	#Twistインスタンス生成

	def image_callback(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')

		#青いラインの検出
		hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)	#色空間の変換(BGR→HSV)
		lower_blue = np.array([90, 64, 0])		#青色の閾値（下限）
		upper_blue = np.array([150, 255, 255])	#青色の閾値（上限）
		mask = cv.inRange(hsv, lower_blue, upper_blue)	#閾値によるHSV画像の2値化（マスク画像生成）
		masked = cv.bitwise_and(image, image, mask = mask)	#mask画像において，1である部分だけが残る（フィルタに通している）

		#黄色い停止線の検出
		lower_yellow1 = np.array([0, 64, 0])		#黄色の閾値（下限）
		upper_yellow1 = np.array([30, 255, 255])	#黄色の閾値（上限）
		mask1 = cv.inRange(hsv, lower_yellow1, upper_yellow1)	#閾値によるHSV画像の2値化（マスク画像生成）
		lower_yellow2 = np.array([150, 64, 0])		#黄色の閾値（下限）
		upper_yellow2 = np.array([180, 255, 255])	#黄色の閾値（上限）
		mask2 = cv.inRange(hsv, lower_yellow2, upper_yellow2)	#閾値によるHSV画像の2値化（マスク画像生成）
		mask_yellow = mask1 + mask2
		masked_yellow = cv.bitwise_and(image, image, mask = mask_yellow)

		h, w = image.shape[:2]
		RESIZE = (w//3, h//3)
		search_top = (h//4)*3
		search_bot = search_top + 20	#目の前の線にだけに興味がある→20行分くらいに絞る
		mask[0:search_top, 0:w] = 0
		mask[search_bot:h, 0:w] = 0

		#分岐
		cx_data = []
		contours, _ = cv.findContours(mask, cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)	#maskにおける1の部分の重心
		for c in contours:
			M = cv.moments(c)
			if M['m00'] > 0:#重心が存在する
				cx = int(M['m10']/M['m00'])	#重心のx座標
				cx_data.append(cx)
				cy = int(M['m01']/M['m00'])	#重心のy座標
				cv.circle(image, (cx, cy), 20, (0, 0, 255), -1)	#赤丸を画像に描画
        
        
		#右パターン
			##P制御
			"""
            cx_max = max(cx_data)
            err = cx_max - w//2	#重心座標(x)と画像の中心(x)との差
			self.twist.linear.x = 0.2
			#self.twist.angular.z = -float(err)/100	#画像が大きいためか，-1/100では絶対値がまだ十分に大きく，ロボットが暴れてしまう
			self.twist.angular.z = -float(err)/1000	#誤差にあわせて回転速度を変化させる（-1/1000がP制御でいうところの比例ゲインにあたる）
			self.cmd_vel_pub.publish(self.twist)"""
		
		#左パターン
			##P制御
			"""
            cx_min = min(cx_data)
            err = cx_min - w//2	#重心座標(x)と画像の中心(x)との差
			self.twist.linear.x = 0.2
			#self.twist.angular.z = -float(err)/100	#画像が大きいためか，-1/100では絶対値がまだ十分に大きく，ロボットが暴れてしまう
			self.twist.angular.z = -float(err)/1000	#誤差にあわせて回転速度を変化させる（-1/1000がP制御でいうところの比例ゲインにあたる）
			self.cmd_vel_pub.publish(self.twist)"""
		
		#停止線処理
		M_yellow = cv.moments(mask_yellow)
		if M_yellow['m00'] > 0:
			self.twist.linear.x = 0.02
			self.twist.angular.z = 0
			self.cmd_vel_pub.publish(self.twist)
			dddd
		ww
		#大きすぎるため，サイズ調整
		display_mask = cv.resize(mask, RESIZE)
		display_masked = cv.resize(masked, RESIZE)
		display_image = cv.resize(image, RESIZE)
		display_mask_yellow = cv.resize(mask_yellow, RESIZE)
		display_masked_yellow = cv.resize(masked_yellow, RESIZE)
		
		#表示
		cv.imshow('BGR Image', display_image)	#'BGR Image'ウィンドウにimageを表示
		cv.imshow('MASK', display_mask)			#'MASK'ウィンドウにimageを表示
		cv.imshow('MASKED', display_masked)		#'MASKED'ウィンドウにimageを表示
		cv.imshow('MASK_yellow', display_mask_yellow)			
		cv.imshow('MASKED_yellow', display_masked_yellow)	
		cv.waitKey(3)	#3ミリ秒待つ

rospy.init_node('follower')	#'follower'という名前でノードを初期化
follower = Follower()	#Followerクラスのインスタンスを作成（init関数が実行される）
rospy.spin()	#ループ