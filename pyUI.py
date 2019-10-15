from PyQt5 import uic 
if __name__ == "__main__":
    with open(".\\ThreeD_viewer.ui") as ui_file:
        with open(".\\ThreeD_viewer.py","w") as py_ui_file:
            uic.compileUi(ui_file,py_ui_file)
    '''
    with open(".\\mesh_parameter.ui") as ui_file:
        with open(".\\mesh_parameter.py","w") as py_ui_file:
            uic.compileUi(ui_file,py_ui_file)
    '''
