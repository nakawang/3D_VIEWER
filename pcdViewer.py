# -*- coding: utf-8 -*-
import vtk,sys,numpy,os
from numpy import random,genfromtxt,size
from PyQt5 import QtCore, QtGui, QtWidgets
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from PyQt5.QtCore import pyqtSlot, QThread
from vtkPointCloud import VtkPointCloud


class PCDviewer(QtWidgets.QFrame):
    def __init__(self, parent, dataPath=None):
        super(PCDviewer,self).__init__(parent)
        self.interactor = QVTKRenderWindowInteractor(self)
        self.layout = QtWidgets.QHBoxLayout()
        self.layout.addWidget(self.interactor)
        self.layout.setContentsMargins(0,0,0,0)
        self.setLayout(self.layout)
        self.pointCloud = VtkPointCloud()
        self.actors = []
        if dataPath != None:
            self.add_newData(dataPath)
    # Renderer
        renderer = vtk.vtkRenderer()
        renderer.AddActor(self.pointCloud.vtkActor)
        #cubeActor = self.addCubeAxesActor(renderer)
        #renderer.AddActor(cubeActor)
    # Scalar Bar
        
        
    #renderer.SetBackground(.2, .3, .4)
        #colors=vtk.vtkNamedColors()
        #colors.SetColor("BkgColor",[179,204,255,255])
        #renderer.SetBackground(colors.GetColor3d("BkgColor"))
        renderer.ResetCamera()
        #renderer.SetLayer(1)
     
    # Render Window
        renderWindow = self.interactor.GetRenderWindow()
        #renderWindow = vtk.vtkRenderWindow()
        print(renderWindow)
        #renderWindow.SetNumberOfLayers(2)
        renderWindow.AddRenderer(renderer)
        #renderWindow.AddRenderer(self.addLogo())
        
        
    # Interactor
        #renderWindowInteractor = vtk.vtkRenderWindowInteractor()
        self.interactor.SetRenderWindow(renderWindow)
        self.interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
    # Scalar Bar
        #self.addScalarBar(self.pointCloud.getLUT())
        #renderer.AddActor(self.addScalarBar(self.pointCloud.getLUT()))
        #renderWindow.SetInteractor(self.interactor)
    # Logo
        #self.addLogo()
    # Begin Interaction
        renderWindow.Render()
        renderWindow.SetWindowName("XYZ Data Viewer:"+ "xyz")
        self.interactor.Start()
        #renderWindowInteractor.Start()
    # Pack to class
        self.renderer=renderer
        #self.interactor=interactor
        #self.xyzLoader.signalOut.connect(self.addActor)
    def start(self):
        self.interactor.Start()

    def addScalarBar(self,lut):
        self.scalarBar = vtk.vtkScalarBarActor()
        self.scalarBar.SetOrientationToVertical()
        self.scalarBar.SetLookupTable(lut)
        self.scalarBar.SetBarRatio(0.12)
        self.scalarBar.SetTitleRatio(0.12)
        self.scalarBar.SetMaximumWidthInPixels(60)
        self.scalarBar.SetMaximumHeightInPixels(300)
        print(self.scalarBar.GetProperty().SetDisplayLocationToBackground())
        #self.scalarBar.SetDisplayPosition(750,250)
        self.scalarBar.SetDisplayPosition(60,200)
        textP = vtk.vtkTextProperty()
        textP.SetFontSize(10)
        self.scalarBar.SetLabelTextProperty(textP)
        self.scalarBar.SetTitleTextProperty(textP)
        self.scalarBar.SetNumberOfLabels(8)
        self.scalarBar.SetLabelFormat("%-#6.3f")#輸出格式
        #self.scalarBarWidget = vtk.vtkScalarBarWidget()
        #self.scalarBarWidget.SetInteractor(self.interactor)
        #self.scalarBarWidget.SetScalarBarActor(self.scalarBar)
        #self.scalarBarWidget.On()
        self.interactor.Initialize()
        return self.scalarBar
    def addCubeAxesActor(self,renderer):
        cubeAxesActor = vtk.vtkCubeAxesActor()
        #設定軸上下限
        cubeAxesActor.SetBounds(self.pointCloud.getBounds())
        #將RENDER CAMERA指定給軸
        cubeAxesActor.SetCamera(renderer.GetActiveCamera())
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
        cubeAxesActor.SetDrawXInnerGridlines(False)
        cubeAxesActor.SetDrawYInnerGridlines(False)
        cubeAxesActor.SetDrawZInnerGridlines(False)
        #網格線顏色
        cubeAxesActor.GetXAxesGridlinesProperty().SetColor(0.5,0.5,0.5)
        cubeAxesActor.GetYAxesGridlinesProperty().SetColor(0.5,0.5,0.5)
        cubeAxesActor.GetZAxesGridlinesProperty().SetColor(0.5,0.5,0.5)
        #控制軸的繪製方式(外,最近,最遠,靜態最近,靜態外)
        cubeAxesActor.SetFlyMode(0)
        #設定刻度線的位置(內,外,兩側)
        cubeAxesActor.SetTickLocation(1)
        #網格線樣式(所有,最近,最遠)
        cubeAxesActor.SetGridLineLocation(2)
        cubeAxesActor.XAxisMinorTickVisibilityOff()
        cubeAxesActor.YAxisMinorTickVisibilityOff()
        cubeAxesActor.ZAxisMinorTickVisibilityOff()
        return cubeAxesActor
    def add_newData(self,path):
        xyz = genfromtxt(path,dtype=float,usecols=[0,1,2])
        minH=xyz[:,2].min()
        maxH=xyz[:,2].max()
        count = len(xyz)
        pcd=VtkPointCloud(minH,maxH,count)
        pcd.clearPoints()
        for k in range(size(xyz,0)):
            point = xyz[k]
            pcd.addPoint(point)
        self.pointCloud = pcd
        self.__addActor()
    def __addActor(self):
        lastActor=self.renderer.GetActors().GetLastActor()
        if lastActor:
            self.renderer.RemoveActor(lastActor)
        actor=self.pointCloud.vtkActor
        #set uniform color
        #actor.GetMapper().ScalarVisibilityOff()
        #actor.GetProperty().SetColor(1.0,0.0,0.0)
        #actor.GetProperty().SetPointSize(4)
        print("set actor color")
        self.renderer.AddActor(actor)
        self.refresh_renderer()
            
    def __removeAll(self):
        actors = self.renderer.GetActors()
        print(actors)
        if len(actors)>0:
            for i in actors:
                self.renderer.RemoveActor(i)
    def refresh_renderer(self):
        render_window=self.interactor.GetRenderWindow()
        self.renderer.ResetCamera()
        render_window.Render()
if __name__=="__main__":
    print("PCD viewer")
