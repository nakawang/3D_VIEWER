# -*- coding: utf-8 -*-
#from fbs_runtime.application_context.PyQt5 import ApplicationContext
import vtk,sys,numpy,os,math
sys.path.append(os.curdir)
from numpy import random,genfromtxt,size
from PyQt5 import QtCore, QtGui, QtWidgets
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from PyQt5.QtCore import pyqtSlot, QThread
from vtkPointCloud import VtkPointCloud
from rawLoader import rawLoader
from pcdViewer import PCDviewer
from plyViewer import PLYviewer
import open3d as o3d
import subprocess
from ExceptionHandle.exceptHandle import *
#import vtk_python_scalarBar
loadPath=os.path.join(os.curdir,"1.xyz")

class XYZviewer(QtWidgets.QFrame):
    def __init__(self, parent, dataPath):
        super(XYZviewer,self).__init__(parent)
        self.interactor = QVTKRenderWindowInteractor(self)
        self.layout = QtWidgets.QHBoxLayout()
        self.layout.addWidget(self.interactor)
        self.layout.setContentsMargins(0,0,0,0)
        self.setLayout(self.layout)
        self.pointCloud = VtkPointCloud()
        self.pcdCollection=[]
        self.actors = []
        self.e=ErrorObserver()
        #self.AddObserver("ErrorEvent",e)
        if self.e.ErrorOccurred():
            print(e.ErrorMessage)
        #self.load_data(loadPath)
        if dataPath != None:
            self.add_data(dataPath)
    # Renderer
        self.renderer = vtk.vtkRenderer()
        self.cubeAxesActor = vtk.vtkCubeAxesActor()
        self.setCubeAxesActor()
        self.cubeAxesActor.SetBounds(0,100,0,100,0,100)
        self.renderer.AddActor(self.cubeAxesActor)
    # Scalar Bar
        self.scalarBarActor = vtk.vtkScalarBarActor()
        self.setScalarBar()
        self.renderer.AddActor(self.scalarBarActor)
    #renderer.SetBackground(.2, .3, .4)
        #colors=vtk.vtkNamedColors()
        #colors.SetColor("BkgColor",[179,204,255,255])
        #renderer.SetBackground(colors.GetColor3d("BkgColor"))
        self.pointCloud.setLUTRange(0,10)
        #cam=self.renderer.GetActiveCamera()
        #cam.Azimuth(-45)
        #cam.Elevation(0)
        #cam.Roll(90)
        #cam.SetViewUp(0,0,1)
        #cam.SetPosition(0,1,0)
        #cam.SetParallelProjection(0)
        #cam.Elevation(-10)
        #self.renderer.SetActiveCamera(cam)
        #self.renderer.ResetCamera()
        #renderer.SetLayer(1)
     
    # Render Window
        renderWindow = self.interactor.GetRenderWindow()
        #renderWindow = vtk.vtkRenderWindow()
        #print(renderWindow)
        #renderWindow.SetNumberOfLayers(2)
        renderWindow.AddRenderer(self.renderer)
        #renderWindow.AddRenderer(self.addLogo())
        
        
    # Interactor
        #renderWindowInteractor = vtk.vtkRenderWindowInteractor()
        self.interactor.SetRenderWindow(renderWindow)
        self.interactor.SetInteractorStyle(vtk.vtkInteractorStyleTerrain())
    # Scalar Bar
        #self.addScalarBar(self.pointCloud.getLUT())
        
        #renderWindow.SetInteractor(self.interactor)
    # Logo
        #self.addLogo()
        self.renderer.ResetCamera()
    # Begin Interaction
        renderWindow.Render()
        renderWindow.SetWindowName("XYZ Data Viewer:"+ "xyz")
        self.interactor.Start()
        #renderWindowInteractor.Start()
    # Pack to class
        #self.renderer=renderer
        #self.interactor=interactor
        
        
    def start(self):
        self.interactor.Start()
    def load_data(self,filename):
        print("start viewer")
        data = genfromtxt(filename,dtype=float,usecols=[0,1,2])
        #print("generate xyz: ",data[0][2])
        #return
        for k in range(size(data,0)):
            point = data[k]
            self.pointCloud.addPoint(point)
    def addLogo(self):
        imgReader = vtk.vtkPNGReader()
        imgReader.SetFileName("benano.png")
        imgReader.Update()
        #print(imgReader.GetOutput())
        imgActor = vtk.vtkImageActor()
        imgActor.SetInputData(imgReader.GetOutput())
        background_renderer = vtk.vtkRenderer()
        background_renderer.SetLayer(0)
        background_renderer.InteractiveOff()
        background_renderer.AddActor(imgActor)
        return background_renderer
    def setScalarBar(self):
        lut=self.pointCloud.getLUT()
        scalarBar=self.scalarBarActor
        scalarBar.SetOrientationToVertical()
        scalarBar.SetLookupTable(lut)
        scalarBar.SetBarRatio(0.12)
        scalarBar.SetTitleRatio(0.12)
        scalarBar.SetMaximumWidthInPixels(60)
        scalarBar.SetMaximumHeightInPixels(300)
        #print(self.scalarBar.GetProperty().SetDisplayLocationToBackground())
        #self.scalarBar.SetDisplayPosition(750,250)
        scalarBar.SetDisplayPosition(60,400)
        textP = vtk.vtkTextProperty()
        textP.SetFontSize(10)
        scalarBar.SetLabelTextProperty(textP)
        scalarBar.SetTitleTextProperty(textP)
        scalarBar.SetNumberOfLabels(8)
        scalarBar.SetLabelFormat("%-#6.3f")#輸出格式
        #self.scalarBarWidget = vtk.vtkScalarBarWidget()
        #self.scalarBarWidget.SetInteractor(self.interactor)
        #self.scalarBarWidget.SetScalarBarActor(self.scalarBar)
        #self.scalarBarWidget.On()
        self.scalarBarActor=scalarBar
    def setCubeAxesActor(self):
        cubeAxesActor = self.cubeAxesActor
        #設定軸上下限
        bounds = self.pointCloud.getBounds()
        cubeAxesActor.SetBounds(bounds)
        #將RENDER CAMERA指定給軸
        cubeAxesActor.SetCamera(self.renderer.GetActiveCamera())
        #設定標題與標籤文字顏色
        cubeAxesActor.GetTitleTextProperty(0).SetColor(0.5,0.5,0.5)
        cubeAxesActor.GetLabelTextProperty(0).SetColor(0.5,0.5,0.5)
        cubeAxesActor.GetTitleTextProperty(1).SetColor(0.5,0.5,0.5)
        cubeAxesActor.GetLabelTextProperty(1).SetColor(0.5,0.5,0.5)
        cubeAxesActor.GetTitleTextProperty(2).SetColor(0.5,0.5,0.5)
        cubeAxesActor.GetLabelTextProperty(2).SetColor(0.5,0.5,0.5)
        #設定坐標軸線寬
        cubeAxesActor.GetXAxesLinesProperty().SetLineWidth(0.5)
        cubeAxesActor.GetYAxesLinesProperty().SetLineWidth(0.5)
        cubeAxesActor.GetZAxesLinesProperty().SetLineWidth(0.5)
        #開啟網格線
        cubeAxesActor.DrawXGridlinesOn()
        cubeAxesActor.DrawYGridlinesOn()
        cubeAxesActor.DrawZGridlinesOn()
        #內部網格線不畫
        cubeAxesActor.SetDrawXInnerGridlines(0)
        cubeAxesActor.SetDrawYInnerGridlines(0)
        cubeAxesActor.SetDrawZInnerGridlines(0)
        #網格線顏色
        cubeAxesActor.GetXAxesGridlinesProperty().SetColor(0.5,0.5,0.5)
        cubeAxesActor.GetYAxesGridlinesProperty().SetColor(0.5,0.5,0.5)
        cubeAxesActor.GetZAxesGridlinesProperty().SetColor(0.5,0.5,0.5)
        #控制軸的繪製方式(外,最近,最遠,靜態最近,靜態外)
        cubeAxesActor.SetFlyMode(4)
        #設定刻度線的位置(內,外,兩側)
        cubeAxesActor.SetTickLocation(1)
        #網格線樣式(所有,最近,最遠)
        cubeAxesActor.SetGridLineLocation(2)
        cubeAxesActor.XAxisMinorTickVisibilityOff()
        cubeAxesActor.YAxisMinorTickVisibilityOff()
        cubeAxesActor.ZAxisMinorTickVisibilityOn()
        self.cubeAxesActor=cubeAxesActor
    def add_axisWidget(self):
        axes = vtk.vtkAxesActor()
        axisWidget = vtk.vtkOrientationMarkerWidget()
        axisWidget.SetOutlineColor(0.9,0.5,0.1)
        axisWidget.SetOrientationMarker(axes)
        iren = self.interactor.GetRenderWindow().GetInteractor()
        axisWidget.SetInteractor(iren)
        axisWidget.SetViewport(0,0,0.4,0.4)
        axisWidget.EnabledOn()
        axisWidget.InteractiveOn()
    def add_newData(self,pcd):
        
        '''
        print("generate xyz")
        for k in range(size(data,0)):
            point = data[k] #20*(random.rand(3)-0.5)
            pcd.addPoint(point)
        self.renderer.AddActor(pcd.vtkActor)
        '''
        self.pointCloud=pcd
        self.addActor()
    def addActor(self):
        """
        self.pcdCollection.append(self.xyzLoader.pcd)
        print("Current pcd count: ", len(self.pcdCollection))
        #self.actors.append(self.pcdCollection[-1].vtkActor)
        #create each actor from xyz collection
        for i in self.pcdCollection:
            self.renderer.AddActor(i.vtkActor)
            #print(i.vtkActor)
        """
        self.removeAll()
        isMesh = False
        isDelaunay3D=False
        isSurfRecon=False
        if isMesh:
            self.pointCloud.generateMesh()
            #self.renderer.AddActor(self.pointCloud.vtkActor)
            self.mainActor=self.pointCloud.boundaryActor
        elif isDelaunay3D:
            self.mainActor=self.pointCloud.delaunay3D()
        elif isSurfRecon:
            self.mainActor=self.pointCloud.surfaceRecon()
        else:
            self.mainActor=self.pointCloud.vtkActor
        self.renderer.AddActor(self.mainActor)
        self.setCubeAxesActor()
        self.renderer.AddActor(self.cubeAxesActor)
        self.setScalarBar()
        self.renderer.AddActor(self.scalarBarActor)
        
        self.renderer.ResetCamera()
        self.refresh_renderer()
        cam = self.renderer.GetActiveCamera()
        self.oriMatrix = cam.GetExplicitProjectionTransformMatrix()
    def removeAll(self):
        actors = self.renderer.GetActors()
        #print(actors)
        for i in actors:
            self.renderer.RemoveActor(i)
        for i in range(len(self.pcdCollection)):
            #print(i)
            del self.pcdCollection[-1]
        #print(len(self.pcdCollection))
    def reset_Camera(self):
        print(self.oriMatrix)
        center_x,center_y,center_z=self.mainActor.GetCenter()
        self.renderer.ResetCamera()
        cam = self.renderer.GetActiveCamera()
        newCam=vtk.vtkCamera()
        newCam.DeepCopy(cam)
        w = vtk.vtkTransform()
        w.RotateX(0.)
        w.RotateY(0.)
        w.RotateZ(0.)
        newCam.SetPosition(center_x,center_y,center_z+1)
        newCam.SetViewUp(0,1,0)
        self.renderer.SetActiveCamera(newCam)
        actors = self.renderer.GetActors()
        for actor in actors:
            actor.SetUserTransform(w)
        self.renderer.ResetCamera()
        self.refresh_renderer()
    def setCameraTop(self):
        center_x,center_y,center_z=self.mainActor.GetCenter()
        cam=self.renderer.GetActiveCamera()
        cam.SetPosition(center_x-1,center_y,center_z)
        cam.SetViewUp(0,0,1)
        #cam.Azimuth(180)
        print(cam.GetPosition())
        #self.renderer.SetActiveCamera(cam)
        self.renderer.ResetCamera()
        self.refresh_renderer()
    def setCameraLeft(self):
        self.renderer.ResetCamera()
        cam=self.renderer.GetActiveCamera()
        #cam.SetPosition(0,0,0)
        #cam.SetViewUp(0,1,0)
        cam.Azimuth(-10)
        #self.renderer.SetActiveCamera(cam)
        self.renderer.ResetCamera()
        self.refresh_renderer()
    def setCameraRight(self):
        self.renderer.ResetCamera()
        cam=self.renderer.GetActiveCamera()
        #cam.SetPosition(0,0,0)
        #cam.SetViewUp(0,1,0)
        cam.Azimuth(10)
        #self.renderer.SetActiveCamera(cam)
        self.renderer.ResetCamera()
        self.refresh_renderer()
    def refresh_renderer(self):
        #self.renderer.ResetCamera()
        renderWindow = self.interactor.GetRenderWindow()
        renderWindow.Render()
    def applyTransform(self,x,y,z):
        center_x,center_y,center_z=self.mainActor.GetCenter()
        w = vtk.vtkTransform()
        #w.Translate(-center_x,-center_y,-center_z)
        #vtk not auto change type from string to double
        w.RotateX(float(x))
        w.RotateY(float(y))
        w.RotateZ(float(z))
        self.mainActor.SetUserTransform(w)
        self.refresh_renderer()
    def setParallelCamera(self,state):
        cam = self.renderer.GetActiveCamera()
        cam.SetParallelProjection(state)
        self.renderer.ResetCamera()
        self.refresh_renderer()
class loaderThread(QThread):
    signalStart = QtCore.pyqtSignal(int)
    signalNow = QtCore.pyqtSignal(int)
    signalOut = QtCore.pyqtSignal()
    def __init__(self, parent=None):
        super().__init__(parent)
        self.fileName = None
        self.pcd = None
        self.rawInfo=None
        self.isWorking=False
    def setFileName(self,fileName):
        self.fileName = fileName
    def __getPointCloud(self):
        
        xyz = genfromtxt(self.fileName,dtype=float,usecols=[0,1,2])
        minH=xyz[:,2].min()
        maxH=xyz[:,2].max()
        count = len(xyz)
        print(count)
        pcd=VtkPointCloud(minH,maxH,count)
        pcd.clearPoints()
        counter=size(xyz,0)
        print(counter)
        self.signalStart.emit(counter)
        print("b")
        for k in range(size(xyz,0)):
            self.signalNow.emit(k)
            point = xyz[k]
            pcd.addPoint(point)
        self.pcd = pcd
        print("b")
    def __getRawToPointCloud(self):
        rLoader=rawLoader()
        rLoader.setRawPath(self.fileName)
        z = rLoader.getHEIGHTRAWVALUE()
        z = z[(z>-99999)]
        zMax = numpy.amax(z)
        zMin = numpy.amin(z)
        #print("get raw:",zMin,zMax)
        count = rLoader.getWIDTH()*rLoader.getHEIGHT()
        pcd = VtkPointCloud(zMin,zMax,count)
        pcd.clearPoints()
        xyz = rLoader.rawToXYZ()
        counter = size(xyz,0)
        self.signalStart.emit(counter)
        
        for i in range(size(xyz,0)):
            self.signalNow.emit(i)
            point = xyz[i]
            pcd.addPoint(point)
        self.rawInfo=[rLoader.getWIDTH(),rLoader.getHEIGHT(),rLoader.getRESX(),rLoader.getRESY(),rLoader.getCHANNEL()]
        self.pcd = pcd
    def run(self):
        if "xyz" in self.fileName:
            print("filename is",self.fileName)
            self.__getPointCloud()
            self.signalOut.emit()
            print("b")
        elif "raw" in self.fileName:
            self.__getRawToPointCloud()
            self.signalOut.emit()
        else:
            return False
    def __del__(self):
        print("loader Thread deleted")
        
class XYZviewerApp(QtWidgets.QMainWindow):
    def __init__(self, dataPath):
        super(XYZviewerApp,self).__init__()
        self.vtk_widget = None
        self.ui = None
        self.dataPath = dataPath
        self.xyzLoader = loaderThread()
        self.setup(self.dataPath)
        
    def setup(self,dataPath):
        import ThreeD_viewer
        self.ui = ThreeD_viewer.Ui_MainWindow()
        self.ui.setupUi(self)
        self.vtk_widget = XYZviewer(self.ui.vtk_panel, self.dataPath)
        self.ui.vtk_layout = QtWidgets.QHBoxLayout()
        self.ui.vtk_layout.addWidget(self.vtk_widget)
        self.ui.vtk_layout.setContentsMargins(0,0,0,0)
        self.ui.vtk_panel.setLayout(self.ui.vtk_layout)
        self.ui.toolButton2.setToolTip("Select xyz file")
        self.ui.toolButton2.clicked.connect(self.on_click)
        self.ui.toolButton1.clicked.connect(self.on_click)
        self.ui.BtnClear.clicked.connect(self.on_click_clear)
        self.ui.BtnClear.setVisible(0)
        self.ui.pushButton2.setVisible(0)
        self.ui.tabWidget.setCurrentIndex(0)
        self.ui.progressBar.setVisible(0)
        self.ui.label_2.setText("")
        #self.setWindowFlags(QtCore.Qt.WindowFullScreen)
        #self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        
        self.tab2_setup()
        
        #logo=QtGui.QPixmap(".\\benano.png")
        #self.ui.label_6.setPixmap(logo)
        #self.ui.label_6.setScaledContents(True)
        #self.ui.label_6.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        #self.ui.label_6.show()
        self.ui.verticalSlider.valueChanged.connect(self.__on_valueChanged)
        self.ui.verticalSlider_2.valueChanged.connect(self.__on_valueChanged)
        self.ui.label_6.setFont(QtGui.QFont("Courier New",40))
        
        self.xyzLoader.signalOut.connect(self.__GetDataFromThread)
        self.xyzLoader.signalStart.connect(self.__onStart_ProgressBar)
        self.xyzLoader.signalNow.connect(self.__onNow_ProgressBar)
        self.xyzLoader.signalOut.connect(self.__onClose_ProgressBar)
        self.showMaximized()
        self.ui.resetCam.clicked.connect(self.__onClicked_ResetCam)
        self.ui.topView.clicked.connect(self.__onClicked_ViewUp)
        self.ui.leftView.clicked.connect(self.__onClicked_ViewLeft)
        self.ui.rightView.clicked.connect(self.__onClicked_ViewRight)
        self.ui.applyTransform.clicked.connect(self.__onClicked_ApplyTransform)
        self.ui.isPerspective.stateChanged.connect(self.__onCheckboxStatusChanged)
    def initialize(self):
        self.vtk_widget.start()
    def tab2_setup(self):
        ind = self.ui.tabWidget.currentIndex()
        print("cur tab:",ind)
        if ind!=0:
            return
        self.pcd_widget = PCDviewer(self.ui.frame_2)
        self.ui.vtk_layout2 = QtWidgets.QHBoxLayout()
        self.ui.vtk_layout2.addWidget(self.pcd_widget)
        self.ui.vtk_layout2.setContentsMargins(0,0,0,0)
        self.ui.frame_2.setLayout(self.ui.vtk_layout2)
        self.ply_widget = PCDviewer(self.ui.frame_3)
        self.ui.vtk_layout3 = QtWidgets.QHBoxLayout()
        self.ui.vtk_layout3.addWidget(self.ply_widget)
        self.ui.vtk_layout3.setContentsMargins(0,0,0,0)
        self.ui.frame_3.setLayout(self.ui.vtk_layout3)
        self.ui.toolButton1_2.clicked.connect(self.on_click)
        self.ui.toolButton1_3.clicked.connect(self.on_click)
        self.ui.PlytoMesh.setVisible(1)
        self.ui.PlytoMesh.setEnabled(False)
        self.ui.PlytoMesh.clicked.connect(self.plytomesh)
        self.ui.SaveMeshTo.setVisible(False)
        self.ui.SaveMeshTo.clicked.connect(self.saveMeshTo)
        self.ui.checkBox.setVisible(1)
        self.ui.checkBox.stateChanged.connect(self.__SyncCamera)
    @pyqtSlot()
    def on_click(self):
        toolName = self.sender().objectName()
        
        if toolName == "toolButton2":
            path,f = QtWidgets.QFileDialog.getOpenFileName(None,"xyz selector", sys.argv[0] , "*.xyz")
            if path:
                self.dataPath = path
                self.refreshViewer()
        elif toolName == "toolButton1":
            path,f = QtWidgets.QFileDialog.getOpenFileName(None,"xyz selector", "D:\\testcode" , "*.raw")
            if path:
                self.dataPath = path
                self.refreshViewer()
        elif toolName == "toolButton1_2":
            path,f = QtWidgets.QFileDialog.getOpenFileName(None,"pointcloud selector", "D:\\testcode" , "(*.xyz);;(*.ply)")
            if path:
                if f=="(*.ply)":
                    ply=o3d.io.read_point_cloud(path)
                    o3d.io.write_point_cloud("temp.xyz",ply)
                    self.pcd_widget.add_newData("temp.xyz")
                    self.dataPath = path
                    self.ui.PlytoMesh.setVisible(True)
                    self.ui.PlytoMesh.setEnabled(True)
                    self.ui.PlytoMesh.setText("PLY to Mesh")
                else:
                    self.pcd_widget.add_newData(path)
        elif toolName == "toolButton1_3":
            path,f = QtWidgets.QFileDialog.getOpenFileName(None,"ply selector", "D:\\testcode" , "*.ply")
            if path:
                self.ply_widget.add_newData(path)
        if path:
            self.__initialBtnStatus()
            print(1)
    @pyqtSlot()
    def on_click_clear(self):
        self.vtk_widget.removeAll()
    def __initialBtnStatus(self):
        self.ui.resetCam.setEnabled(1)
        self.ui.leftView.setEnabled(1)
        self.ui.rightView.setEnabled(1)
        self.ui.topView.setEnabled(1)
        self.ui.applyTransform.setEnabled(1)
        self.ui.verticalSlider.setEnabled(1)
        self.ui.verticalSlider_2.setEnabled(1)
        print(2)
    def __ResetSlider(self):
        bounds=self.vtk_widget.pointCloud.getBounds()
        zMin=bounds[4]
        zMax=bounds[5]
        r=zMax-zMin
        lutMin,lutMax=self.vtk_widget.pointCloud.lut.GetTableRange()
        sliderMin = (lutMin-zMin)/r*100
        sliderMax = (lutMax-zMin)/r*100
        print(lutMin,lutMax,sliderMin,sliderMax)
        if sliderMax > zMax:
            sliderMax = 100
        self.ui.verticalSlider_2.setValue(100)
        self.ui.verticalSlider.setValue(0)
        self.__on_valueChanged()
    def __on_valueChanged(self):
        if self.ui.verticalSlider.value()>=self.ui.verticalSlider_2.value():
            self.ui.verticalSlider.setValue(self.ui.verticalSlider_2.value())
        if self.ui.verticalSlider_2.value()<=self.ui.verticalSlider.value():
            self.ui.verticalSlider_2.setValue(self.ui.verticalSlider.value())
        bounds=self.vtk_widget.pointCloud.getBounds()
        zMin=bounds[4]
        zMax=bounds[5]
        r=zMax-zMin
        max=self.ui.verticalSlider_2.value()*r/100+zMin
        min=self.ui.verticalSlider.value()*r/100+zMin
        print(max,min)
        self.vtk_widget.pointCloud.setLUTRange(min,max)
        #self.vtk_widget.reset_Camera()
        self.vtk_widget.refresh_renderer()
    def __GetDataFromThread(self):
        pcd = self.xyzLoader.pcd
        self.vtk_widget.add_newData(pcd)
        self.__ResetSlider()
        if "raw" in self.xyzLoader.fileName:
            self.__UpdateText(self.xyzLoader.rawInfo)
    def addPCDData(self):
        self.pcd_widget.add_newData()
    def addPLYData(self):
        self.ply_widget.add_newData()
    def refreshViewer(self):
        print("1")
        print(self.dataPath)
        self.xyzLoader.setFileName(self.dataPath)
        self.xyzLoader.start()
    def plytomesh(self):
        bType="2"
        mWidth="0.25"
        pWeight="0"
        
        p = os.path.join(os.curdir,"tempMesh.ply")
        print(p)
        if os.path.isfile(p):
            os.remove(p)
        print("mesh generator")
        poisson="PoissonRecon.exe"
        file = self.dataPath
        outfile = p
        arg=r" --in "+file+" --out "+outfile+" --bType " +bType+" --width "+mWidth+" --pointWeight "+pWeight+" --fullDepth --colors"
        subprocess.call(poisson+arg,shell=True)
        #while cp.returnCode
        self.ply_widget.add_newData(outfile)
        self.ui.SaveMeshTo.setVisible(1)
        self.ui.checkBox.setVisible(1)
    def saveMeshTo(self):
        path,f = QtWidgets.QFileDialog.getSaveFileName(None,"Save file", "D:\\" , "*.ply;;*.stl")
        ply=o3d.io.read_triangle_mesh("tempMesh.ply")
        if path:
            o3d.io.write_triangle_mesh(path,ply)
            o3d.io.write_triangle_mesh("123.stl",ply,write_ascii = True)
            return
    def __UpdateText(self,rawInfo):
        txt = rawInfo
        w,h,x,y=str(txt[0]),str(txt[1]),str(txt[2]),str(txt[3])
        self.ui.label_2.setFont(QtGui.QFont("Courier New",12))
        self.ui.label_2.setText("Raw Width:"+w+"\n"+"Raw Height:"+h+"\n"+"Resolution X:"+x+"\n"+"Resolution Y:"+y+"\n")
    def __SyncCamera(self,state):
        obs1=None
        obs2=None
        if state==2:
            obs1=self.pcd_widget.interactor.AddObserver("ModifiedEvent",self.modifiedCallback)
            obs2=self.ply_widget.interactor.AddObserver("ModifiedEvent",self.modifiedCallback)
            cam=vtk.vtkCamera()
            self.pcd_widget.renderer.SetActiveCamera(cam)
            self.pcd_widget.renderer.ResetCamera()
            self.ply_widget.renderer.SetActiveCamera(cam)
            self.ply_widget.renderer.ResetCamera()
        else:
            print("cancel sync camera: ",obs1,obs2)
            self.pcd_widget.interactor.RemoveObserver(obs1)
            self.ply_widget.interactor.RemoveObserver(obs2)
            cam0=vtk.vtkCamera()
            cam1=vtk.vtkCamera()
            self.pcd_widget.renderer.SetActiveCamera(cam0)
            self.ply_widget.renderer.SetActiveCamera(cam1)
            self.pcd_widget.renderer.ResetCamera()
            self.ply_widget.renderer.ResetCamera()
    def __onStart_ProgressBar(self,i):
        self.ui.progressBar.setVisible(1)
        self.ui.progressBar.setRange(0,i)
    def __onNow_ProgressBar(self,value):
        self.ui.progressBar.setValue(value)
    def __onClose_ProgressBar(self):
        self.ui.progressBar.setVisible(0)
    def modifiedCallback(self,obj,ev):
        self.ply_widget.interactor.GetRenderWindow().Render()
        self.pcd_widget.interactor.GetRenderWindow().Render()
        return
    def __onClicked_ResetCam(self):
        self.vtk_widget.reset_Camera()
    def __onClicked_ViewUp(self):
        self.vtk_widget.setCameraTop()
    def __onClicked_ViewLeft(self):
        self.vtk_widget.setCameraLeft()
    def __onClicked_ViewRight(self):
        self.vtk_widget.setCameraRight()
    def __onClicked_ApplyTransform(self):
        thetaX=self.ui.thetaX.toPlainText()
        thetaY=self.ui.thetaY.toPlainText()
        thetaZ=self.ui.thetaZ.toPlainText()
        self.vtk_widget.applyTransform(thetaX,thetaY,thetaZ)
        print(thetaX,thetaY,thetaZ)
    def __onCheckboxStatusChanged(self,state):
        self.vtk_widget.setParallelCamera(state)
def getFilePath():
    if len(sys.argv) >2:
         print('Usage: xyzviewer.py itemfile')
         sys.exit()
    return sys.argv[1]
if __name__ == '__main__':
    #xyzViewer = XYZviewer("D:\\TestCode\\tutorial-vtk-pyqt-master\\volume\\Export.xyz")
    #xyzViewer.start()
    #p="D:\\TestCode\\tutorial-vtk-pyqt-master\\volume\\Export.xyz"
    file=open(".\\Eclippy\\Eclippy.qss","r")
    
    #appctxt = ApplicationContext()
    #qss=appctxt.getResource(".\\Eclippy\\Eclippy.qss")
    #poissonRecon=appctxt.getResource(".\\PoissonRecon.exe")
    #file = open(qss,"r")
    app = QtWidgets.QApplication([])
    with file:
        qss=file.read()
        app.setStyleSheet(qss)
    main_window = XYZviewerApp(None)
    main_window.initialize()
    main_window.show()
    print("end")
    #exit_code = appctxt.app.exec_()      # 2. Invoke appctxt.app.exec_()
    #sys.exit(exit_code)
    
    
