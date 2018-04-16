
import cv2
import numpy as np

def Features():
    global CadernoDetectado
    MIN_MATCH_COUNT=50

    detector= cv2.xfeatures2d.SIFT_create()   #########

    FLANN_INDEX_KDITREE=0
    flannParam=dict(algorithm=FLANN_INDEX_KDITREE,tree=5)
    flann=cv2.FlannBasedMatcher(flannParam,{})

    trainImg=cv2.imread("Caderno.jpeg",0)
    trainKP,trainDesc=detector.detectAndCompute(trainImg,None)

    raposa = 0

    font = cv2.FONT_HERSHEY_SIMPLEX

    cam=cv2.VideoCapture(0)
    while True:
        ret, QueryImgBGR=cam.read()
        QueryImg=cv2.cvtColor(QueryImgBGR,cv2.COLOR_BGR2GRAY)
        queryKP,queryDesc=detector.detectAndCompute(QueryImg,None) ########
        matches=flann.knnMatch(queryDesc,trainDesc,k=2)   ########

        goodMatch=[]
        for m,n in matches:
            if(m.distance<0.75*n.distance):
                goodMatch.append(m)
        if(len(goodMatch)>MIN_MATCH_COUNT):
            tp=[]
            qp=[]
            for m in goodMatch:
                tp.append(trainKP[m.trainIdx].pt)
                qp.append(queryKP[m.queryIdx].pt)
            tp,qp=np.float32((tp,qp))
            H,status=cv2.findHomography(tp,qp,cv2.RANSAC,3.0)
            h,w=trainImg.shape
            trainBorder=np.float32([[[0,0],[0,h-1],[w-1,h-1],[w-1,0]]])
            queryBorder=cv2.perspectiveTransform(trainBorder,H)
            cv2.polylines(QueryImgBGR,[np.int32(queryBorder)],True,(0,255,0),5)
            CadernoDetectado = True

        else:
            CadernoDetectado = False
            
            
            
            
        gray = cv2.medianBlur(cv2.cvtColor(cam.read()[1], cv2.COLOR_BGR2GRAY),5)
        
        if(raposa==1):
            cv2.putText(QueryImgBGR, "ACHOU!!!!!", (230, 50), font, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
            print("ACHOU!!")
        cv2.imshow('result',QueryImgBGR)
        if cv2.waitKey(10)==ord('q'):
            break
        


    cam.release()
    cv2.destroyAllWindows()
Features()