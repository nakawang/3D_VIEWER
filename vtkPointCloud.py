import vtk,sys
import numpy as np
from numpy import random
from vtk.util import numpy_support
class VtkPointCloud:
    def __init__(self, zMin=-100.0, zMax=100.0, maxNumPoints=5e7):
        self.maxNumPoints = maxNumPoints
        self.vtkPolyData = vtk.vtkPolyData()
        self.clearPoints()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(self.vtkPolyData)
        mapper.SetColorModeToDefault()
        #mapper.SetColorModeToDirectScalars()
        mapper.SetScalarRange(zMin, zMax)
        mapper.SetScalarVisibility(1)
        
        lut = vtk.vtkLookupTable()
        lut.SetHueRange(0.667,0)
        #lut.SetTableRange(0,1)
        lut.SetAboveRangeColor(255,255,255,1)
        lut.SetBelowRangeColor(0,0,0,0)
        lut.UseBelowRangeColorOn()
        lut.UseAboveRangeColorOn()
        lut.Build()
        mapper.SetLookupTable(lut)
        mapper.SetUseLookupTableScalarRange(True)
        self.mapper=mapper
        self.vtkActor = vtk.vtkActor()
        self.vtkActor.SetMapper(mapper)
        self.lut = lut
    def addPoint(self, point):
        if self.vtkPoints.GetNumberOfPoints() <self.maxNumPoints:
            pointId = self.vtkPoints.InsertNextPoint(point[:])
            self.vtkDepth.InsertNextValue(point[2])
            self.vtkCells.InsertNextCell(1)
            self.vtkCells.InsertCellPoint(pointId)
        else:
            r = random.randint(0, self.maxNumPoints)
            self.vtkPoints.SetPoint(r, point[:])
        self.vtkCells.Modified()
        self.vtkPoints.Modified()
        self.vtkDepth.Modified()
    def setPoints(self,a,b,c,d):
        print("1")
        #self.vtkPolyData.SetPoints(vtkVerts)
        print("1")
        #self.vtkPolyData.SetVerts(vtkCellArray)
        print("1")
        self.vtkPoints.SetData(a)
        self.vtkCells.SetCells(b,c)
        self.vtkCells.Modified()
        self.vtkPoints.Modified()
        print("1")
        self.vtkPolyData.GetPointData().SetScalars(d)
    def clearPoints(self):
        self.vtkPoints = vtk.vtkPoints()
        self.vtkCells = vtk.vtkCellArray()
        self.vtkDepth = vtk.vtkDoubleArray()
        self.vtkDepth.SetName('DepthArray')
        self.vtkPolyData.SetPoints(self.vtkPoints)
        self.vtkPolyData.SetVerts(self.vtkCells)
        self.vtkPolyData.GetPointData().SetScalars(self.vtkDepth)
        self.vtkPolyData.GetPointData().SetActiveScalars('DepthArray')
    def getBounds(self):
        return self.vtkPoints.GetBounds()
    def setRGBColor(self,colorArr):
        self.vtkPolyData.GetPointData().SetScalars(colorArr)
        print("Set color: ",colorArr)
    def getLUT(self):
        return self.lut
    def setLUTRange(self,min,max):
        self.lut.SetTableRange(min,max)
    def delaunay2D(self):
        print("start generate mesh")
        boundary = vtk.vtkPolyData()
        boundary.SetPoints(self.vtkPolyData.GetPoints())
        aCellArray=vtk.vtkCellArray()
        boundary.SetPolys(aCellArray)
        delaunay=vtk.vtkDelaunay2D()
        delaunay.SetSourceData(boundary)
        delaunay.SetInputData(self.vtkPolyData)
        delaunay.Update()
        print("finish delaunay")
        meshMapper=vtk.vtkPolyDataMapper()
        meshMapper.SetInputConnection(delaunay.GetOutputPort())
        meshMapper.SetLookupTable(self.lut)
        meshMapper.SetScalarVisibility(1)
        meshMapper.SetUseLookupTableScalarRange(True)
        self.vtkActor = vtk.vtkActor()
        self.vtkActor.SetMapper(meshMapper)
        self.vtkActor.GetProperty().SetEdgeColor(0, 0, 1)
        self.vtkActor.GetProperty().SetInterpolationToFlat()
        self.vtkActor.GetProperty().SetRepresentationToWireframe()
        boundaryMapper = vtk.vtkPolyDataMapper()
        boundaryMapper.SetInputData(boundary)
        boundaryActor = vtk.vtkActor()
        boundaryActor.SetMapper(boundaryMapper)
        boundaryActor.GetProperty().SetColor(1, 0, 0)
        self.boundaryActor = vtk.vtkActor()
    def delaunay3D(self):
        delny = vtk.vtkDelaunay3D()
        delny.SetInputData(self.vtkPolyData)
        delny.SetTolerance(0.01)
        delny.SetAlpha(0.2)
        delny.BoundingTriangulationOff()
        #shrink = vtk.vtkShrinkFilter()
        #shrink.SetInputConnection(delny.GetOutputPort())
        #shrink.SetShrinkFactor(0.9)
        mapper=vtk.vtkDataSetMapper()
        mapper.SetInputConnection(delny.GetOutputPort())
        triangulation=vtk.vtkActor()
        triangulation.SetMapper(mapper)
        triangulation.GetProperty().SetColor(1,0,0)
        return triangulation
    def surfaceRecon(self):
        pointSource=vtk.vtkProgrammableSource()
        def readPoints():
            output = pointSource.GetPolyDataOutput()
            #points = vtk.vtkPoints()
            output.SetPoints(self.vtkPoints)
        pointSource.SetExecuteMethod(readPoints)
        surf = vtk.vtkSurfaceReconstructionFilter()
        surf.SetInputConnection(pointSource.GetOutputPort())
        print(surf)
        contour = vtk.vtkContourFilter()
        contour.SetInputConnection(surf.GetOutputPort())
        contour.SetValue(0,0.0)
        print(contour)
        reverse = vtk.vtkReverseSense()
        reverse.SetInputConnection(contour.GetOutputPort())
        reverse.ReverseCellsOn()
        reverse.ReverseNormalsOn()
        contourMapper=vtk.vtkPolyDataMapper()
        contourMapper.SetInputConnection(reverse.GetOutputPort())
        contourMapper.ScalarVisibilityOff()
        print(contourMapper)
        contourActor=vtk.vtkActor()
        contourActor.SetMapper(contourMapper)
        print(contourActor)
        return contourActor
    def setRTFilter(self,rt):
        print("ori:",self.vtkPolyData)
        if 0:
            rt=vtk.vtkTransform()
            rt.RotateX(90)
            rt.RotateY(5)
        rtFilter=vtk.vtkTransformPolyDataFilter()
        rtFilter.SetInputData(self.vtkPolyData)
        rtFilter.SetTransform(rt)
        rtFilter.Update()
        #self.mapper.SetInputConnection(rtFilter.GetOutputPort())
        self.vtkPolyData=rtFilter.GetOutput()
        
        print("abc:",self.vtkPolyData)
        points=self.vtkPolyData.GetPoints()
        print("new data points:",points,points.GetNumberOfPoints())
        #for i in range(points.GetNumberOfPoints()):
            #print(points.GetPoint(i))
        print(self.vtkPolyData.GetPointData().GetArray(0))
        #x=points.GetNumberOfPoints()+1
        #self.vtkPolyData.GetPointData().GetArray(0)
        #get numpy array of vtk array
        np_points=numpy_support.vtk_to_numpy(points.GetData())
        depth = np_points[:,2]
        print(np_points)
        print("new depth",depth)
        self.vtkPoints=points
        self.vtkDepth = numpy_support.numpy_to_vtk(depth)
        self.vtkPolyData.GetPointData().SetScalars(self.vtkDepth)
        self.mapper.SetInputData(self.vtkPolyData)
        print("rt done")
