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
		lower_yellow = np.array([90, 64, 10])		#aoの閾値（下限）
		upper_yellow = np.array([150, 255, 255])	#ao色の閾値（上限）
		mask1 = cv.inRange(hsv, lower_yellow, upper_yellow)	#閾値によるHSV画像の2値化（マスク画像生成）
		mask2 = cv.inRange(hsv, lower_yellow, upper_yellow)	#閾値によるHSV画像の2値化（マスク画像生成）
		mask3 = cv.inRange(hsv, lower_yellow, upper_yellow)	#閾値によるHSV画像の2値化（マスク画像生成）
		h, w = image.shape[:2] #(w, h) =(800, 600)
		RESIZE = (w//3, h//3) #RESIZE = (266, 200)
		search_top = (h//4)*3 #search_top = 450 
		search_bot_1 = search_top + 40
		search_bot_2 = search_top + 20	#目の前の線にだけに興味がある→20行分くらいに絞る
		search_bot_3 = search_top - 20

		#mask1 = mask
		#mask2 = mask これじゃあできないよ
		#mask3 = mask

		mask1[0:search_bot_2, 0:w] = 0
		mask1[search_bot_1:h, 0:w] = 0

		mask2[0:search_top, 0:w] = 0
		mask2[search_bot_2:h, 0:w] = 0

		mask3[0:search_bot_3, 0:w] = 0
		mask3[search_top:h, 0:w] = 0

		masked = cv.bitwise_and(image, image, mask = mask2)	#mask画像において，1である部分だけが残る（フィルタに通している）

		M1 = cv.moments(mask1)	#mask1における1の部分の重心
		M2 = cv.moments(mask2)	#mask2における1の部分の重心
		M3 = cv.moments(mask3)	#mask3における1の部分の重心

		cx1, cy1, cx2, cy2, cx3, cy3 = 0, 0, 0, 0, 0, 0

		if M1['m00'] > 0:	#重心が存在する
			cx1 = int(M1['m10']/M1['m00'])	#重心のx座標
			cy1 = int(M1['m01']/M1['m00'])	#重心のy座標
		if M2['m00'] > 0:	#重心が存在する
			cx2 = int(M2['m10']/M2['m00'])	#重心のx座標
			cy2 = int(M2['m01']/M2['m00'])	#重心のy座標
		if M3['m00'] > 0:	#重心が存在する
			cx3 = int(M3['m10']/M3['m00'])	#重心のx座標
			cy3 = int(M3['m01']/M3['m00'])	#重心のy座標

		#3点の重心の近似曲線
		
		data = np.array([[cx1, cy1], [cx2, cy2], [cx3, cy3]])
		k = 0
		datanew = []
		#重心が取れていない場合、dataから(0,0)を抜く
		while k<3:
			if data[k][0] == 0 and data[k][1] == 0:
				k+=1
			else:
				datanew.append([data[k][0], data[k][1]])
				k+=1

		datanew = np.array(datanew)
		x = datanew[:, 0]
		y = datanew[:, 1]
		a, b = np.polyfit(x, y, 1)
		y_new = cy2
		x_new = int(((y_new-b)/a)//1)
		cv.circle(image, (x_new,y_new), 20, (0, 0, 255), -1)	#赤丸を画像に描画
		#cv.line(image, (w-1, a), (0,b), (0, 255, 0), 2)
		
		##P制御
		err = x_new - w//2	#黄色の先の重心座標(x)と画像の中心(x)との差
		self.twist.linear.x = 0.2
		#self.twist.angular.z = -float(err)/100	#画像が大きいためか，-1/100では絶対値がまだ十分に大きく，ロボットが暴れてしまう
		self.twist.angular.z = -float(err)/100	#誤差にあわせて回転速度を変化させる（-1/1000がP制御でいうところの比例ゲインにあたる）
		self.cmd_vel_pub.publish(self.twist)
		

		#大きすぎるため，サイズ調整
		display_mask = cv.resize(mask2, RESIZE)
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

