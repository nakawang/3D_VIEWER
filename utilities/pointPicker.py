import vtk 
class testStyle(vtk.vtkInteractorStyleTrackballCamera):
    def __init__(self,cbFunc,rbFunc,parent=None):
        self.AddObserver("LeftButtonPressEvent",self.leftButtonPressEvent)
        self.AddObserver("RightButtonPressEvent",self.rightButtonPressEvent)
        self.LastPickedActor = None
        self.LastPickedProperty = vtk.vtkProperty()
        self.cbFunc=cbFunc
        self.rbFunc=rbFunc
    def leftButtonPressEvent(self,obj,event):
        print(obj)
        clickPos = self.GetInteractor().GetEventPosition()
        print(clickPos)
        #picker = self.GetInteractor().GetPicker()
        #print("Get current picker: ", picker)
        picker = vtk.vtkPointPicker()
        picker.SetTolerance(0.001)
        ren = self.GetInteractor().GetRenderWindow().GetRenderers().GetFirstRenderer()
        picker.Pick(clickPos[0],clickPos[1],0,ren)
        print(picker.GetPointId())
        self.cbFunc(picker.GetPointId())
    def rightButtonPressEvent(self,obj,event):
        print(obj)
        self.rbFunc()

if __name__=="__main__":
    #a=testStyle()
    print("testStyle")
