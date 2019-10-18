import vtk

class ErrorObserver:

   def __init__(self):
       self.__ErrorOccurred = False
       self.__ErrorMessage = None
       self.CallDataType = 'string0'

   def __call__(self, obj, event, message):
       self.__ErrorOccurred = True
       self.__ErrorMessage = message

   def ErrorOccurred(self):
       occ = self.__ErrorOccurred
       self.__ErrorOccurred = False
       return occ

   def ErrorMessage(self):
       return self.__ErrorMessage

if __name__=="__main__":
   e = ErrorObserver()

   a = vtk.vtkImageReader()
   a.AddObserver('ErrorEvent', e)

   print("doing update")
   a.Update()
   if e.ErrorOccurred():
      print(e.ErrorMessage())
