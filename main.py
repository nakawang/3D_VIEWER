# -*- coding: utf-8 -*-
#from fbs_runtime.application_context.PyQt5 import ApplicationContext
import vtk,sys,numpy,os,math,glob
import pandas as pd
sys.path.append(os.curdir)
from numpy import loadtxt,size
from PyQt5 import QtCore, QtGui, QtWidgets
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from PyQt5.QtCore import pyqtSlot, QThread, QTimer
from vtkPointCloud import VtkPointCloud
from rawLoader import rawLoader
from pcdViewer import PCDviewer
from plyViewer import PLYviewer
from vtk.util import numpy_support
import open3d as o3d
import subprocess
from utilities.projectPlane import *
from ExceptionHandle.exceptHandle import *
#import vtk_python_scalarBar
loadPath=os.path.join(os.curdir,"1.xyz")
vtk.vtkObject.GlobalWarningDisplayOff()
class XYZviewer(QtWidgets.QFrame):
    pickedPointSignal = QtCore.pyqtSignal(str)
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
        self.pickedID=[]
        self.e=ErrorObserver()
        self.interactor.AddObserver("ErrorEvent",self.e)
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
        self.interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
    # Scalar Bar
        #self.addScalarBar(self.pointCloud.getLUT())
        
        #renderWindow.SetInteractor(self.interactor)
    # Logo
        #self.addLogo()
    # orientation marker
        self.addAxisWidget()
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
        cubeAxesActor.GetXAxesLinesProperty().SetLineWidth(1)
        cubeAxesActor.GetYAxesLinesProperty().SetLineWidth(1)
        cubeAxesActor.GetZAxesLinesProperty().SetLineWidth(1)
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
        cubeAxesActor.SetFlyMode(3)
        #設定刻度線的位置(內,外,兩側)
        cubeAxesActor.SetTickLocation(1)
        #網格線樣式(所有,最近,最遠)
        cubeAxesActor.SetGridLineLocation(1)
        cubeAxesActor.XAxisMinorTickVisibilityOff()
        cubeAxesActor.YAxisMinorTickVisibilityOff()
        cubeAxesActor.ZAxisMinorTickVisibilityOn()
        self.cubeAxesActor=cubeAxesActor
    def addAxisWidget(self):
        axes = vtk.vtkAxesActor()
        self.axisWidget = vtk.vtkOrientationMarkerWidget()
        self.axisWidget.SetOutlineColor(0.9,0.5,0.1)
        self.axisWidget.SetOrientationMarker(axes)
        self.axisWidget.SetInteractor(self.interactor)
        #self.axisWidget.SetViewport(0,0,0.4,0.4)
        self.axisWidget.EnabledOn()
        self.axisWidget.InteractiveOff()
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
        print("start add actor")
        self.removeAll()
        isMesh = False
        isDelaunay3D=False
        isSurfRecon=False
        isDelaunay2D=False
        if isMesh:
            self.pointCloud.generateMesh()
            #self.renderer.AddActor(self.pointCloud.vtkActor)
            self.mainActor=self.pointCloud.boundaryActor
        elif isDelaunay3D:
            self.mainActor=self.pointCloud.delaunay3D()
        elif isSurfRecon:
            self.mainActor=self.pointCloud.surfaceRecon()
        elif isDelaunay2D:
            delny = vtk.vtkDelaunay2D()
            print("1")
            delny.SetInputData(self.pointCloud.vtkPolyData)
            print("1")
            delny.SetSourceData(self.pointCloud.vtkPolyData)
            print("1")
            mapper = vtk.vtkPolyDataMapper()
            print("1")
            mapper.SetInputConnection(delny.GetOutputPort())
            mapper.SetColorModeToDefault()
            print("1")
            actor=vtk.vtkActor()
            print("1")
            actor.SetMapper(mapper)
            print("1")
            self.mainActor=actor
        else:
            self.mainActor=self.pointCloud.vtkActor
        print(self.pointCloud)
        self.renderer.AddActor(self.mainActor)
        self.setCubeAxesActor()
        self.renderer.AddActor(self.cubeAxesActor)
        self.setScalarBar()
        self.renderer.AddActor(self.scalarBarActor)
        
        self.renderer.ResetCamera()
        self.refresh_renderer()
        cam = self.renderer.GetActiveCamera()
        self.oriMatrix = cam.GetExplicitProjectionTransformMatrix()
        print("Add actor done.")
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
        cam = self.renderer.GetActiveCamera()
        cam.SetPosition(center_x,center_y,center_z+100)
        cam.SetViewUp(0,1,0)
        self.renderer.ResetCamera()
        self.refresh_renderer()
    def setCameraTop(self):
        center_x,center_y,center_z=self.mainActor.GetCenter()
        cam=self.renderer.GetActiveCamera()
        cam.SetPosition(center_x+1,center_y,center_z)
        cam.SetViewUp(0,0,1)
        cam.Azimuth(90)
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
    def applyXTransform(self,x):
        w = vtk.vtkTransform()
        w.RotateX(float(x))
        self.pointCloud.setRTFilter(w)
        self.addActor()
        self.refresh_renderer()
    def applyYTransform(self,y):
        w = vtk.vtkTransform()
        w.RotateX(float(y))
        self.pointCloud.setRTFilter(w)
        self.addActor()
        self.refresh_renderer()
    def applyZTransform(self,z):
        w = vtk.vtkTransform()
        w.RotateX(float(z))
        self.pointCloud.setRTFilter(w)
        self.addActor()
        self.refresh_renderer()
    def applyTransform(self,x,y,z):
        center_x,center_y,center_z=self.mainActor.GetCenter()
        w = vtk.vtkTransform()
        #w.Translate(-center_x,-center_y,-center_z)
        #vtk not auto change type from string to double
        w.RotateX(float(x))
        w.RotateY(float(y))
        w.RotateZ(float(z))
        #self.mainActor.SetUserTransform(w)
        #self.scalarBarActor.SetUserTransform(w)
        #self.cubeAxesActor.SetUserTransform(w)
        self.pointCloud.setRTFilter(w)
        self.addActor()
        self.refresh_renderer()
    def setParallelCamera(self,state):
        cam = self.renderer.GetActiveCamera()
        cam.SetParallelProjection(state)
        self.renderer.ResetCamera()
        self.refresh_renderer()
    def setPickerMode(self,state):
        import utilities.pointPicker as pStyle
        print(pStyle)
        print(state)
        if state==2:
            self.interactor.SetInteractorStyle(pStyle.testStyle(self.emitPickedPoint,self.removePickedPoints))
        else:
            self.interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
    def emitPickedPoint(self,pointId):
        self.pickedID.append(pointId)
        x,y,z=self.pointCloud.vtkPoints.GetPoint(pointId)
        print("emit:",pointId,x,y,z)
        px = "{:.3f}".format(x)
        py = "{:.3f}".format(y)
        pz = "{:.3f}".format(z)
        txt = "Picked:"+px+","+py+","+pz
        self.pickedPointSignal.emit(txt)
        sphereSource = vtk.vtkSphereSource()
        sphereSource.SetCenter(x,y,z)
        sphereSource.SetRadius(1)
        sphereSource.SetThetaResolution(10)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphereSource.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(1,0,0)
        #actor.GetProperty().SetRepresentationToWireframe()
        print(actor)
        actors = self.renderer.GetActors()
        print(actors)
        self.renderer.AddActor(actor)
        self.refresh_renderer()
        if len(self.pickedID)>3:
            #remove draw line 20191104
            return
            self.drawKochanekSpline(self.pickedID)
    def removePickedPoints(self):
        count = len(self.pickedID)
        print("picked count:",count)
        if count>0:
            for i in range(count):
                actors = self.renderer.GetActors()
                self.renderer.RemoveActor(actors.GetLastActor())
        self.pickedID=[]
        self.refresh_renderer()
    def projectionPlane(self):
        b=self.pointCloud.getBounds()
        print("get boundary",b)
        print("create projection plane")
        projectPoints =  projectXYplane(b[4]-1,self.pointCloud.vtkPoints)
        pPointsY=projectXZplane(b[2]-1,self.pointCloud.vtkPoints)
        pPointsX=projectYZplane(b[0]-1,self.pointCloud.vtkPoints)
        print("y boundary",b[2])
        '''
        pZ = vtk.vtkPlane()
        pZ.SetOrigin(0,0,b[4]-1)
        pZ.SetNormal(0,0,1)
        print("create projection points")
        projectPoints = vtk.vtkPoints()
        print("set projection points")
        for i in range(self.pointCloud.vtkPoints.GetNumberOfPoints()):
            p=self.pointCloud.vtkPoints.GetPoint(i)
            #print("pts:",p)
            projectedPoint=numpy.zeros(3)
            pZ.ProjectPoint(p,projectedPoint)
            #print("pts:",projectedPoint)
            projectPoints.InsertNextPoint(projectedPoint)
        '''
        print("Get projected points",projectPoints)
        aaa = vtk.vtkPolyData()
        print("Add point source",aaa)
        aaa.SetPoints(projectPoints)
        abc=vtk.vtkPolyData()
        abc.SetPoints(pPointsY)
        ccc=vtk.vtkPolyData()
        ccc.SetPoints(pPointsX)
        '''
        create 2d delaunay mesh
        delny = vtk.vtkDelaunay2D()
        delny.SetTolerance(0.01)
        delny.SetInputData(aaa)
        delny.SetSourceData(aaa)
        '''
        
        '''
        #create edges
        extract = vtk.vtkExtractEdges()
        extract.SetInputConnection(delny.GetOutputPort())
        '''
        vertexGlyphFilter = vtk.vtkVertexGlyphFilter()
        vertexGlyphFilter.SetInputData(aaa)
        vertexGlyphFilter.Update()
        vertexGlyphFilter1 = vtk.vtkVertexGlyphFilter()
        vertexGlyphFilter1.SetInputData(abc)
        vertexGlyphFilter1.Update()
        vertexGlyphFilter2 = vtk.vtkVertexGlyphFilter()
        vertexGlyphFilter2.SetInputData(ccc)
        vertexGlyphFilter2.Update()
        print("Add mapper")
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(vertexGlyphFilter.GetOutputPort())
        mapper1 = vtk.vtkPolyDataMapper()
        mapper1.SetInputConnection(vertexGlyphFilter1.GetOutputPort())
        mapper2 = vtk.vtkPolyDataMapper()
        mapper2.SetInputConnection(vertexGlyphFilter2.GetOutputPort())
        print("set Actor")
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0,0,1)
        actor1 = vtk.vtkActor()
        actor1.SetMapper(mapper1)
        actor1.GetProperty().SetColor(0,1,0)
        actor2 = vtk.vtkActor()
        actor2.SetMapper(mapper2)
        actor2.GetProperty().SetColor(1,0,0)
        print("Add Actor",actor)
        self.renderer.AddActor(actor)
        self.renderer.AddActor(actor1)
        self.renderer.AddActor(actor2)
        self.refresh_renderer()
        print("refresh renderer")
    def drawParametricSpline(self,IDList):
        points = vtk.vtkPoints()
        for i in IDList:
            p=self.pointCloud.vtkPoints.GetPoint(i)
            points.InsertNextPoint(p)
        spline = vtk.vtkParametricSpline()
        spline.SetPoints(points)
        functionSource = vtk.vtkParametricFunctionSource()
        functionSource.SetParametricFunction(spline)
        functionSource.Update()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(functionSource.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        self.renderer.AddActor(actor)
        self.refresh_renderer()
    def drawKochanekSpline(self,IDList):
        points = vtk.vtkPoints()
        for i in IDList:
            p=self.pointCloud.vtkPoints.GetPoint(i)
            points.InsertNextPoint(p)
        xSpline = vtk.vtkKochanekSpline()
        ySpline = vtk.vtkKochanekSpline()
        zSpline = vtk.vtkKochanekSpline()
        spline = vtk.vtkParametricSpline()
        spline.SetXSpline(xSpline)
        spline.SetYSpline(ySpline)
        spline.SetZSpline(zSpline)
        spline.SetPoints(points)
        functionSource = vtk.vtkParametricFunctionSource()
        functionSource.SetParametricFunction(spline)
        functionSource.Update()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(functionSource.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        self.renderer.AddActor(actor)
        self.refresh_renderer()
    def SurfaceReconstruction(self):
        pointSource=vtk.vtkProgrammableSource()
        def readPoints():
            output = pointSource.GetPolyDataOutput()
            points = vtk.vtkPoints()
            output.SetPoints(points)
            for i in IDList:
                p=self.pointCloud.vtkPoints.GetPoint(i)
                points.InsertNextPoint(p)
        pointSource.SetExecuteMethod(readPoints)
        surf = vtk.vtkSurfaceReconstructionFilter()
        surf.SetInputConnection(pointSource.GetOutputPort())
        cf = vtk.vtkContourFilter()
        cf.SetInputConnection(surf.GetOutputPort())
        cf.SetValue(0,0)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(cf.GetOutputPort())
        mapper.ScalarVisibilityOff()
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetDiffuseColor(1,0.3882,0.2784)
        actor.GetProperty().SetSpecularColor(1,1,1)
        actor.GetProperty().SetSpecular(.4)
        actor.GetProperty().SetSpecularPower(50)
        self.renderer.AddActor(actor)
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
        #self.qMutex=QMutexLocker()
    def setFileName(self,fileName):
        self.fileName = fileName
    def __getPointCloud(self):
        self.signalStart.emit(0)
        print("b")
        xyz = pd.read_csv(self.fileName,header=None,delim_whitespace=True,dtype=float,usecols=[0,1,2])
        rgb = pd.read_csv(self.fileName,header=None,delim_whitespace=True,dtype=float,usecols=[3,4,5])
        xyz = xyz.values
        print(xyz)
        theta1=numpy.radians(35)
        theta2=numpy.radians(5)
        c1,c2,s1,s2=numpy.cos(theta1),numpy.cos(theta2),numpy.sin(theta1),numpy.sin(theta2)
        rx=numpy.array(((1,0,0),(0,c1,-s1),(0,s1,c1)))
        ry=numpy.array(((c2,0,s2),(0,1,0),(-s2,0,c2)))
        xyz=xyz.dot(rx)
        print(xyz)
        xyz=xyz.dot(ry)
        rgb = rgb.values
        print(xyz,rgb)
        #xyz = loadtxt(self.fileName,dtype=float,usecols=[0,1,2])
        minH=xyz[:,2].min()
        maxH=xyz[:,2].max()
        print("minh,maxh:",minH,maxH)
        count = len(xyz)
        print(len(xyz.shape))
        pcd=VtkPointCloud(minH,maxH,count)
        pcd.clearPoints()
        counter=size(xyz,0)
        print(counter)
        #test np to vtk array
        nCoords = xyz.shape[0]
        nElem = xyz.shape[1]
        print("c,e",nCoords,nElem)
        print("xyz",xyz)
        depth = xyz[:,2]
        print("depth",depth)
        colors = numpy_support.numpy_to_vtk(rgb)
        vtkDepth = numpy_support.numpy_to_vtk(depth)
        cells_npy = numpy.vstack([numpy.ones(nCoords,dtype=numpy.int64),
                               numpy.arange(nCoords,dtype=numpy.int64)]).T.flatten()
        print("cells_npy",cells_npy)
        cells = vtk.vtkCellArray()
        cells.SetCells(nCoords,numpy_support.numpy_to_vtkIdTypeArray(cells_npy))
        print("cells",cells)
        vtkArray=numpy_support.numpy_to_vtk(xyz)
        verts = vtk.vtkPoints()
        verts.SetData(vtkArray)
        print(vtkArray)
        pcd.setPoints(vtkArray,nCoords,numpy_support.numpy_to_vtkIdTypeArray(cells_npy),vtkDepth)
        #pcd.setRGBColor(colors)
        '''
        for k in range(size(xyz,0)):
            self.signalNow.emit(k)
            point = xyz[k]
            pcd.addPoint(point)
        '''
        self.pcd = pcd
        print("b")
    def __getRawToPointCloud(self):
        self.signalStart.emit(0)
        rLoader=rawLoader()
        rLoader.setRawPath(self.fileName)
        z = rLoader.getHEIGHTRAWVALUE()
        z = z[(z>-99999)]
        zMax = numpy.amax(z)
        zMin = numpy.amin(z)
        print("get raw:",zMin,zMax)
        count = rLoader.getWIDTH()*rLoader.getHEIGHT()
        pcd = VtkPointCloud(zMin,zMax,count)
        pcd.clearPoints()
        xyz = rLoader.rawToXYZ()
        #================================================
        '''
        downsample by open3d
        '''
        print("donwsample by open3d")
        p3d = o3d.geometry.PointCloud()
        p3d.points=o3d.utility.Vector3dVector(xyz)
        print("np to open3d")
        downpcd = o3d.voxel_down_sample(p3d,voxel_size=0.05)
        xyz=numpy.asarray(downpcd.points)
        #================================================
        counter = size(xyz,0)
        for i in range(size(xyz,0)):
            point = xyz[i]
            pcd.addPoint(point)
            self.signalNow.emit(i)
        self.rawInfo=[rLoader.getWIDTH(),rLoader.getHEIGHT(),rLoader.getRESX(),rLoader.getRESY(),rLoader.getCHANNEL()]
        self.pcd = pcd
    def run(self):
        try:
            if ".xyz" in self.fileName:
                print("filename is",self.fileName)
                self.__getPointCloud()
                self.signalOut.emit()
                print("b")
            elif ".raw" in self.fileName:
                self.__getRawToPointCloud()
                self.signalOut.emit()
            else:
                return False
            self.finished.emit()
        except:
            print("something wrong")
            return False
    def __del__(self):
        print("loader Thread deleted")
        self.wait()
        
class XYZviewerApp(QtWidgets.QMainWindow):
    def __init__(self, dataPath):
        super(XYZviewerApp,self).__init__()
        self.vtk_widget = None
        self.ui = None
        self.dataPath = dataPath
        self.setup(self.dataPath)
        #self.autoLoader = monitorData()
        self.isLoading=False
        self.monitorFolder=None
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
        self.ui.toolButton.clicked.connect(self.on_click)
        self.ui.BtnClear.clicked.connect(self.on_click_clear)
        self.ui.BtnClear.setVisible(0)
        self.ui.pushButton2.setVisible(0)
        self.ui.tabWidget.setCurrentIndex(0)
        self.ui.progressBar.setVisible(0)
        self.ui.detailView.setText("")
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.monitorData)
        #self.setWindowFlags(QtCore.Qt.WindowFullScreen)
        #self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        self.xyzLoader = loaderThread()
        self.tab2_setup()
        
        #logo=QtGui.QPixmap(".\\benano.png")
        #self.ui.label_6.setPixmap(logo)
        #self.ui.label_6.setScaledContents(True)
        #self.ui.label_6.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        #self.ui.label_6.show()
        self.ui.verticalSlider.valueChanged.connect(self.__on_valueChanged)
        self.ui.verticalSlider_2.valueChanged.connect(self.__on_valueChanged)
        self.ui.label_6.setFont(QtGui.QFont("Courier New",40))
        self.ui.label_2.setText("")
        self.ui.label_2.setFont(QtGui.QFont("Courier New",8))
        self.xyzLoader.signalOut.connect(self.__GetDataFromThread)
        self.xyzLoader.signalStart.connect(self.__onStart_ProgressBar)
        #self.xyzLoader.signalNow.connect(self.__onNow_ProgressBar)
        #self.xyzLoader.signalOut.connect(self.__onClose_ProgressBar)
        self.xyzLoader.signalOut.connect(self.__initialBtnStatus)
        self.showMaximized()
        self.ui.resetCam.clicked.connect(self.__onClicked_ResetCam)
        self.ui.topView.clicked.connect(self.__onClicked_ViewUp)
        self.ui.leftView.clicked.connect(self.__onClicked_ViewLeft)
        self.ui.rightView.clicked.connect(self.__onClicked_ViewRight)
        #self.ui.applyTransform.clicked.connect(self.__onClicked_ApplyTransform)
        self.ui.isPerspective.stateChanged.connect(self.__onCheckboxStatusChanged)
        self.ui.pointPicker.stateChanged.connect(self.__onPickerMode)
        self.vtk_widget.pickedPointSignal.connect(self.__printPickedPoint)
        self.ui.autoLoading.stateChanged.connect(self.__onAutoLoading)
        self.ui.pushButton_2.clicked.connect(self.vtk_widget.projectionPlane)
        self.ui.toolButton_2.clicked.connect(self.__onClicked_ApplyXTransform)
        self.ui.toolButton_6.clicked.connect(self.__onClicked_ApplyYTransform)
        self.ui.toolButton_8.clicked.connect(self.__onClicked_ApplyZTransform)
        self.ui.toolButton_3.clicked.connect(self.__onClicked_ApplymXTransform)
        self.ui.toolButton_7.clicked.connect(self.__onClicked_ApplymYTransform)
        self.ui.toolButton_9.clicked.connect(self.__onClicked_ApplymZTransform)

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
            path,f = QtWidgets.QFileDialog.getOpenFileName(None,"xyz selector", "D:\\" , "*.xyz")
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
        elif toolName == "toolButton":
            fd = QtWidgets.QFileDialog()
            fd.setFileMode(4)
            path= fd.getExistingDirectory()
            if path:
                self.ui.label_2.setText(path)
                self.__printPickedPoint(path)
                self.monitorFolder=path
    @pyqtSlot()
    def on_click_clear(self):
        self.vtk_widget.removeAll()
    def __initialBtnStatus(self):
        self.ui.resetCam.setEnabled(1)
        self.ui.leftView.setEnabled(1)
        self.ui.rightView.setEnabled(1)
        self.ui.topView.setEnabled(1)
        #self.ui.applyTransform.setEnabled(1)
        self.ui.verticalSlider.setEnabled(1)
        self.ui.verticalSlider_2.setEnabled(1)
        self.ui.pointPicker.setEnabled(1)
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
        '''
        while(self.xyzLoader.isFinished()):
            print("wait finished.")
        if self.xyzLoader.isFinished():
            pcd = self.xyzLoader.pcd
        else:
            print("not finished.")
            return
        '''
        pcd = self.xyzLoader.pcd
        self.vtk_widget.add_newData(pcd)
        self.__ResetSlider()
        if "raw" in self.xyzLoader.fileName:
            self.__UpdateText(self.xyzLoader.rawInfo)
        self.__onClose_ProgressBar()
        self.isLoading=False
    def addPCDData(self):
        self.pcd_widget.add_newData()
    def addPLYData(self):
        self.ply_widget.add_newData()
    def refreshViewer(self):
        self.xyzLoader.setFileName(self.dataPath)
        print(self.dataPath)
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
        self.ui.detailView.setFont(QtGui.QFont("Courier New",12))
        self.ui.detailView.setPlainText("Raw Width:"+w+"\n"+"Raw Height:"+h+"\n"+"Resolution X:"+x+"\n"+"Resolution Y:"+y+"\n")
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
        self.ui.progressBar.setRange(0,0)
        self.isLoading=True
    def __onNow_ProgressBar(self,value):
        self.ui.progressBar.setValue(value)
    def __onClose_ProgressBar(self):
        self.ui.progressBar.setVisible(0)
        #self.isLoading=False
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
    '''
    def __onClicked_ApplyTransform(self):
        thetaX=self.ui.thetaX.toPlainText()
        thetaY=self.ui.thetaY.toPlainText()
        thetaZ=self.ui.thetaZ.toPlainText()
        self.vtk_widget.applyTransform(thetaX,thetaY,thetaZ)
        print(thetaX,thetaY,thetaZ)
    '''
    def __onClicked_ApplyXTransform(self):
        thetaX=self.ui.lineEdit.text()
        self.vtk_widget.applyTransform(thetaX,0,0)
    def __onClicked_ApplyYTransform(self):
        thetaY=self.ui.lineEdit_2.text()
        self.vtk_widget.applyTransform(0,thetaY,0)
    def __onClicked_ApplyZTransform(self):
        thetaZ=self.ui.lineEdit_3.text()
        self.vtk_widget.applyTransform(0,0,thetaZ)
    def __onClicked_ApplymXTransform(self):
        thetaX=self.ui.lineEdit.text()
        self.vtk_widget.applyTransform(float(thetaX)*-1,0,0)
    def __onClicked_ApplymYTransform(self):
        thetaY=self.ui.lineEdit_2.text()
        self.vtk_widget.applyTransform(0,float(thetaY)*-1,0)
    def __onClicked_ApplymZTransform(self):
        thetaZ=self.ui.lineEdit_3.text()
        self.vtk_widget.applyTransform(0,0,float(thetaZ)*-1)
        
    def __onCheckboxStatusChanged(self,state):
        self.vtk_widget.setParallelCamera(state)
    def __onPickerMode(self,state):
        self.vtk_widget.setPickerMode(state)
    def __printPickedPoint(self,idNo):
        txt=self.ui.detailView.toPlainText()
        txt=txt+idNo+"\n"
        print(idNo)
        self.ui.detailView.setText(txt)
        cursor = self.ui.detailView.textCursor()
        cursor.movePosition(11)
        self.ui.detailView.setTextCursor(cursor)
    def __onAutoLoading(self,state):
        #Watching new file generation
        print("auto loader state: ", state)
        if state==2:
            self.timer.start(1500)
            print("start auto loading")
            self.ui.toolButton1.setEnabled(0)
            self.ui.toolButton2.setEnabled(0)
        else:
            self.timer.stop()
            self.ui.toolButton1.setEnabled(1)
            self.ui.toolButton2.setEnabled(1)
        return
    def monitorData(self):
        try:
            if self.isLoading:
                print("still loading")
                return
            path=self.monitorFolder
            print("start check file")
            if path==None:
                print("check path exists")
                self.__printPickedPoint("Please set a monitor folder.")
                self.__printPickedPoint("Stop monitor.")
                self.ui.autoLoading.setChecked(False)
                return
            path=path+"/*.xyz"
            files=glob.glob(path)
            print("check file counts",len(files))
            if len(files)<1:
                print("file count zero")
                return 
            latest_file=max(files,key=os.path.getmtime)
            print("check last file name",latest_file)
            print(files)
            if self.dataPath != latest_file:
                self.isLoading=True
                print("start load:",latest_file)
                txt="Load file: "+latest_file
                self.__printPickedPoint(txt)
                self.dataPath=latest_file
                self.refreshViewer()
            elif isTest:
                self.isLoading=True
                print("start load:",latest_file)
                txt="Load file: "+latest_file
                self.__printPickedPoint(txt)
                self.dataPath=latest_file
                self.refreshViewer()
            print("wait next check")
        except:
            print("crash")
    def closeEvent(self,event):
        print("app closed")
        self.timer.stop()
        self.xyzLoader.exit()
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
    isTest=False
    app = QtWidgets.QApplication([])
    with file:
        qss=file.read()
        app.setStyleSheet(qss)
    file.close()
    main_window = XYZviewerApp(None)
    main_window.initialize()
    main_window.show()
    print("end")
    #exit_code = appctxt.app.exec_()      # 2. Invoke appctxt.app.exec_()
    #sys.exit(exit_code)
    
    
