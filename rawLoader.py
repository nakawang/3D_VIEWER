import numpy as np
import array
import open3d as o3d
class rawLoader:
    __VERSION=""
    __WIDTH=""
    __HEIGHT=""
    __RESX=""
    __RESY=""
    __CHANNEL=""
    __PATH=""
    __HEIGHTRAWVALUE=""
    __XYZ=None
    def setRawPath(self,path):
        self.__PATH = path
        self.__getRawInfo()
    def __getRawInfo(self):
        f=open(self.__PATH,"rb")
        info = np.frombuffer(f.read(16),dtype=np.int)
        imgResolution = np.frombuffer(f.read(16),dtype=np.float)
        np_arr = np.fromfile(f,dtype=np.float32)
        self.__VERSION=info[0]
        self.__WIDTH=info[1]
        self.__HEIGHT=info[2]
        self.__RESX=imgResolution[0]
        self.__RESY=imgResolution[1]
        self.__CHANNEL=info[3]
        self.__HEIGHTRAWVALUE = np_arr
        f.close()
    def getWIDTH(self):
        return self.__WIDTH
    def getHEIGHT(self):
        return self.__HEIGHT
    def getRESX(self):
        return self.__RESX
    def getRESY(self):
        return self.__RESY
    def getCHANNEL(self):
        return self.__CHANNEL
    def getHEIGHTRAWVALUE(self):
        return self.__HEIGHTRAWVALUE
    def getXYZ(self):
        return self.__XYZ
    def rawToXYZ(self):
        xyz = np.zeros((self.__HEIGHTRAWVALUE.size,3))
        np_x = np.arange(self.__WIDTH)
        np_x = np.tile(np_x,self.__HEIGHT)
        np_y = np.arange(self.__HEIGHT)
        np_y = np.repeat(np_y,self.__WIDTH)
        realX = np_x * (self.__RESX)/1000
        realY = np_y * (self.__RESY)/1000
        xyz[:,0]=realX
        xyz[:,1]=realY*-1
        xyz[:,2]=self.__HEIGHTRAWVALUE
        self.__XYZ = xyz[np.all(xyz >-99998,axis=1)]
        #self.outputXYZ("d:\\TestCode\\metalCenter\\test.xyz")
        return self.__XYZ
    def outputXYZ(self,dest):
        pcd = o3d.geometry.PointCloud()
        if self.__XYZ.any():
            pcd.points = o3d.utility.Vector3dVector(self.__XYZ)
            o3d.io.write_point_cloud(dest,pcd)
            return True
        else:
            return False
if __name__=="__main__":
    print("BENANO RAW LOADER")
        
        
