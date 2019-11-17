import vtk,numpy
def projectXYplane(planeZ,vtkPoints):
    print("create xy projection")
    pZ=vtk.vtkPlane()
    pZ.SetOrigin(0,0,planeZ)
    pZ.SetNormal(0,0,1)
    projectPoints=vtk.vtkPoints()
    print("1")
    for i in range(vtkPoints.GetNumberOfPoints()):
        p=vtkPoints.GetPoint(i)
        projectedPoint=numpy.zeros(3)
        pZ.ProjectPoint(p,projectedPoint)
        projectPoints.InsertNextPoint(projectedPoint)
    print("2")
    return projectPoints

def projectYZplane(planeX,vtkPoints):
    pX=vtk.vtkPlane()
    pX.SetOrigin(planeX,0,0)
    pX.SetNormal(1,0,0)
    projectPoints=vtk.vtkPoints()
    for i in range(vtkPoints.GetNumberOfPoints()):
        p=vtkPoints.GetPoint(i)
        projectedPoint=numpy.zeros(3)
        pX.ProjectPoint(p,projectedPoint)
        projectPoints.InsertNextPoint(projectedPoint)
    return projectPoints

def projectXZplane(planeY,vtkPoints):
    pY=vtk.vtkPlane()
    pY.SetOrigin(0,planeY,0)
    pY.SetNormal(0,1,0)
    projectPoints=vtk.vtkPoints()
    for i in range(vtkPoints.GetNumberOfPoints()):
        p=vtkPoints.GetPoint(i)
        projectedPoint=numpy.zeros(3)
        pY.ProjectPoint(p,projectedPoint)
        projectPoints.InsertNextPoint(projectedPoint)
    return projectPoints
