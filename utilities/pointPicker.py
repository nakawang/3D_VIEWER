import vtk 
class testStyle(vtk.vtkInteractorStyleTrackballCamera):
    def __init__(self,cbFunc,parent=None):
        self.AddObserver("LeftButtonPressEvent",self.leftButtonPressEvent)
        self.LastPickedActor = None
        self.LastPickedProperty = vtk.vtkProperty()
        self.cbFunc=cbFunc
    def leftButtonPressEvent(self,obj,event):
        print(obj)
        clickPos = self.GetInteractor().GetEventPosition()
        print(clickPos)
        #picker = self.GetInteractor().GetPicker()
        #print("Get current picker: ", picker)
        picker = vtk.vtkPointPicker()
        ren = self.GetInteractor().GetRenderWindow().GetRenderers().GetFirstRenderer()
        picker.Pick(clickPos[0],clickPos[1],0,ren)
        print(picker.GetPointId())
        self.cbFunc(picker.GetPointId())

if __name__=="__main__":
    a=testStyle()
    print("testStyle")
