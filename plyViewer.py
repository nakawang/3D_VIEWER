# -*- coding: utf-8 -*-
import vtk,sys,numpy,os
from numpy import random,genfromtxt,size
from PyQt5 import QtCore, QtGui, QtWidgets
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from PyQt5.QtCore import pyqtSlot, QThread
from vtkPointCloud import VtkPointCloud

class PLYviewer(QtWidgets.QFrame):
    def __init__(self,parent,dataPath=None):
        super(PLYviewer,self).__init__(parent)
        if dataPath!=None:
            self.add_newData(dataPath)
        else:
            self.plyReader=vtk.vtkPLYReader()
        self.interactor=QVTKRenderWindowInteractor(self)
        self.layout=QtWidgets.QHBoxLayout()
        self.layout.addWidget(self.interactor)
        self.layout.setContentsMargins(0,0,0,0)
        self.setLayout(self.layout)
        renderer=vtk.vtkRenderer()
        renderer.ResetCamera()
        renderWindow=self.interactor.GetRenderWindow()
        renderWindow.AddRenderer(renderer)
        self.interactor.SetRenderWindow(renderWindow)
        self.interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
        self.interactor.Initialize()
        renderWindow.Render()
        self.interactor.Start()
        self.renderer=renderer
        self.renderWindow=renderWindow
    def add_newData(self,path):
        print("get ply")
        reader=vtk.vtkPLYReader()
        #reader.SetDuplicatePointsForFaceTexture(False)
        reader.SetFileName(path)
        reader.Update()
        self.plyReader = reader
        self.__addActor()
    def __testGlyph3d(self):
        glyph = vtk.vtkVertexGlyphFilter()
        glyph.SetInputConnection(self.plyReader.GetOutputPort())
        #glyph.ScalingOn()
        glyph.ClampingOn()
        glyph.Update()
    def __addActor(self):
        self.__removeAll()
        print("1")
        plyMapper = vtk.vtkPolyDataMapper()
        print(self.plyReader.GetOutput())
        plyMapper.SetInputConnection(self.plyReader.GetOutputPort())
        plyMapper.SetColorModeToDirectScalars()
        plyMapper.SetUseLookupTableScalarRange(1)
        lut = vtk.vtkLookupTable()
        plyMapper.SetLookupTable(lut)
        print("2")
        plyActor = vtk.vtkActor()
        plyActor.SetMapper(plyMapper)
        print("3")
        self.renderer.AddActor(plyActor)
        self.refresh_renderer()
    def refresh_renderer(self):
        self.renderer.ResetCamera()
        self.renderWindow.Render()
        print("done")
    def __removeAll(self):
        actors = self.renderer.GetActors()
        for i in actors:
            self.renderer.RemoveActor(i)
        self.refresh_renderer()
        
if __name__=="__main__":
    print("PLYviewer")
