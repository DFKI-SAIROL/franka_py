import numpy as np

class MessageHandler:
    """Base class for message handlers."""
    def __init__(self, fields=None):
        self.fields = fields

    def extract(self, msg):
        raise NotImplementedError

class JointStateHandler(MessageHandler):
    def extract(self, msg):
        """
        Extracts joint state data.
        Returns a dictionary with keys: position, velocity, effort (if present/requested).
        """
        data = {}
        # Default fields if none specified
        fields_to_extract = self.fields if self.fields else ['position', 'velocity', 'effort']
        
        for field in fields_to_extract:
            if hasattr(msg, field):
                val = getattr(msg, field)
                # Convert to numpy array immediately
                data[field] = np.array(val, dtype=np.float64)
        
        return data

class PoseStampedHandler(MessageHandler):
    def extract(self, msg):
        """
        Extracts PoseStamped data.
        Returns flattened position and orientation.
        """
        data = {}
        
        # Position
        if not self.fields or 'position' in self.fields:
            data['position'] = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float64)
            
        # Orientation
        if not self.fields or 'orientation' in self.fields:
            data['orientation'] = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype=np.float64)
            
        return data

class ImageHandler(MessageHandler):
    def extract(self, msg):
        """
        Extracts Image data.
        """
        # simplified extraction assuming relatively standard encoding or raw bytes
        #Ideally use cv_bridge, but we can do a naive copy for now if we want to avoid deps, 
        # but for real usage we usually want the array.
        
        # We will try to just return the buffer as a flat array reshaped? 
        # Without cv_bridge, interpreting encoding is hard.
        # But users often want raw data.
        
        # Let's try to do a basic conversion from 'data' (bytes) to numpy
        dtype = np.uint8
        if '16UC1' in msg.encoding:
            dtype = np.uint16
        elif '32FC1' in msg.encoding:
            dtype = np.float32

        # Warning: This is a simplistic conversion. 
        # Ideally we should use cv_bridge, but let's stick to simple flexible extraction.
        # If user wants strict cv2, they can add it. 
        # For now, we save the raw bytes as numpy array or reshaped if we know dims.
        
        arr = np.frombuffer(msg.data, dtype=dtype)
        
        if self.fields and 'flatten' in self.fields:
             return {'image': arr}
             
        try:
            # Attempt reshape if step/width/height are sane
            channels = 1
            if 'rgb' in msg.encoding or 'bgr' in msg.encoding:
                channels = 3
            elif 'rgba' in msg.encoding or 'bgra' in msg.encoding:
                channels = 4
                
            arr = arr.reshape((msg.height, msg.width, channels))
        except:
            pass # Keep flat if reshape fails
            
        return {'image': arr}

def get_handler(msg_type, fields=None):
    if 'JointState' in msg_type:
        return JointStateHandler(fields)
    elif 'PoseStamped' in msg_type:
        return PoseStampedHandler(fields)
    elif 'Image' in msg_type:
        return ImageHandler(fields)
    else:
        # Generic handler fallback?
        return None
