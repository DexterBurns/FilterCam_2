import ids_peak.ids_peak as some_alias
from ids_peak import ids_peak
from ids_peak import ids_peak_ipl_extension
from ids_peak_ipl import ids_peak_ipl
from PIL import Image
import os 
import datetime
import time
import sys

# change to serial number of the camera
SERIALNO = "4108867964"

doo = sys.argv

for el in sys.argv:
    print(el)
    
# Variable 
received_filter_num = sys.argv[1]

" Set Brightness, exposure and white balance to continuous for now. "

def main():

    # Initialize SDK
    ids_peak.Library.Close()
    ids_peak.Library.Initialize()

    # Create a DeviceManager object
    device_manager = ids_peak.DeviceManager.Instance()

    try:
        # Update the DeviceManager
        device_manager.Update()

        # Exit program if no device was found
        if device_manager.Devices().empty():
            print("No device found. Exiting Program.")
            return -1

        # Iterate over the list of devices
        for device in device_manager.Devices():
            serial = device.SerialNumber()
            if serial == SERIALNO:
                print("found device")
                
        # Open device if found
        descriptor = device_manager.Devices()[0]
        device = descriptor.OpenDevice(ids_peak.DeviceAccessType_Control)
        
        # Get the nodemaps for the device
        nodemap = device.RemoteDevice().NodeMaps()[0]
        
        # Load settings we want on startup
        LoadCameraSettings(nodemap)
        
        # Init a DataStream 
        datastream = DataStreamInit(device)
        
        # init buffer
        buffer = BufferInit(nodemap, datastream)
              
        # Start image Acquisition
        AcquireImageFromBuffer(datastream, nodemap)


    # Break if any error/exception found
    except Exception as e:
        print("EXCEPTION: " + str(e))
        return -2

    finally:
        ids_peak.Library.Close()
        print("Device closed")

# Loads the settings we want for the camera
def LoadCameraSettings(remote_nodemap):
    
    remote_nodemap.FindNode("UserSetSelector").SetCurrentEntry("Default")
    remote_nodemap.FindNode("UserSetLoad").Execute()
    remote_nodemap.FindNode("UserSetLoad").WaitUntilDone()
    
    # Auto white balance, Auto gain, auto exposure
    auto_exposure_status = remote_nodemap.FindNode("ExposureAuto").CurrentEntry().SymbolicValue()
    auto_gain_status = remote_nodemap.FindNode("GainAuto").CurrentEntry().SymbolicValue()
    
    if auto_exposure_status == "Off":
        remote_nodemap.FindNode("ExposureAuto").SetCurrentEntry("Continuous")
        
    if auto_gain_status == "Off":
        remote_nodemap.FindNode("GainAuto").SetCurrentEntry("Continuous")
        
     
def GetTime():
    
    time_stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return time_stamp


# Init the data stream
def DataStreamInit(device):
    
    datastream = device.DataStreams()[0].OpenDataStream()
    
    return datastream

def BufferInit(nodemap, datastream):
    
    payload_size = nodemap.FindNode("PayloadSize").Value()
    buffer_count_max = datastream.NumBuffersAnnouncedMinRequired()
    
    # Allocate buffers and add them to the pool
    for buffer_count in range(buffer_count_max):
        # Let the TL allocate the buffers
        buffer = datastream.AllocAndAnnounceBuffer(payload_size)
        # Put the buffer in the pool
        datastream.QueueBuffer(buffer)
     
    return buffer

def AcquireImageFromBuffer(data_stream, remote_nodemap):
    
    # Folder for the images to be saved in
    timestamp = GetTime()
    folder_name = f"/home/dexter/ROS2/FilterCam/src/camera_scripts/Snapshots/images_{timestamp}"
    
    # Lock writeable nodes during acquisition
    remote_nodemap.FindNode("TLParamsLocked").SetValue(1)

    print("Starting acquisition...")
    data_stream.StartAcquisition()
    remote_nodemap.FindNode("AcquisitionStart").Execute()
    remote_nodemap.FindNode("AcquisitionStart").WaitUntilDone()
    
    # This loop is so that autoexposure and other settings can converge. So we throw away the initial frames.
    for junk in range(20):
    
        try:
            # Wait for finished/filled buffer event
            buffer = data_stream.WaitForFinishedBuffer(3000)
            img = ids_peak_ipl_extension.BufferToImage(buffer)
            
            # Put the buffer back in the pool, so it can be filled again
            # NOTE: If you want to use `img` beyond this point, you have
            #       to make a copy, since `img` still uses the underlying
            #       buffer's memory.
            data_stream.QueueBuffer(buffer)

            
        except Exception as e:
            print(f"Exception: {e}")
        
    # This loop takes the actual photo    
    for index in range(10):
        
        try:
            # Wait for finished/filled buffer event
            buffer = data_stream.WaitForFinishedBuffer(1000)
            img = ids_peak_ipl_extension.BufferToImage(buffer)

            # Do something with `img` here ...
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            os.makedirs(folder_name, exist_ok=True)
            
            # Put the buffer back in the pool, so it can be filled again
            # NOTE: If you want to use `img` beyond this point, you have
            #       to make a copy, since `img` still uses the underlying
            #       buffer's memory.
            data_stream.QueueBuffer(buffer)
            
            # Saving the image to a folder in the workspace
            timestamp = GetTime()
            file = "Capture_" + str(index) + "_" + str(timestamp) + "_Filter_" + str(received_filter_num) + ".png"
            
            full_path_to_save_img = folder_name + "/" + file
            
            ids_peak_ipl.ImageWriter.Write(full_path_to_save_img, img)
            
            
            
        except Exception as e:
            print(f"Exception: {e}")
            sys.exit(-1)

    print("Stopping acquisition...")
    remote_nodemap.FindNode("AcquisitionStop").Execute()
    remote_nodemap.FindNode("AcquisitionStop").WaitUntilDone()

    data_stream.StopAcquisition(ids_peak.AcquisitionStopMode_Default)

    # In case another thread is waiting on WaitForFinishedBuffer
    # you can interrupt it using:
    # data_stream.KillWait()

    # Remove buffers from any associated queue
    data_stream.Flush(ids_peak.DataStreamFlushMode_DiscardAll)

    for buffer in data_stream.AnnouncedBuffers():
        # Remove buffer from the transport layer
        data_stream.RevokeBuffer(buffer)

    # Unlock writeable nodes again
    remote_nodemap.FindNode("TLParamsLocked").SetValue(0)
 

main()

#Exit on success
sys.exit(0)


