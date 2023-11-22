import React, { useContext } from "react";
import { ServiceRequest } from "roslib";
import { ROSContext, ROSProvider } from "../context/ROSContext";
import { 
  ACTION_SERVICE_ID, 
  CONTROLLER_TYPE_SERVICE_ID,
  PID_TUNING_SERVICE_ID 
} from "../config";

function useROS() {
  const [
    ros, 
    actionService, 
    controllerTypeService,
    pidTuningService, 
    setROS
] = useContext(ROSContext);

  function changeUrl(new_url) {
    setROS((ros) => ({ ...ros, url: new_url }));
  }

  const handleConnect = () => {
    try {
      ros.ROS.connect(ros.url);

      ros.ROS.on("connection", () => {
        setROS((ros) => ({ 
          ...ros, 
          isConnected: true, 
          error: null 
        }));
      });

      ros.ROS.on("close", () => {
        setROS((ros) => ({ 
          ...ros, 
          isConnected: false,
          error: null
        }));
      });

      ros.ROS.on("error", (error) => {
        setROS((ros) => ({ ...ros, error }));
      });
    } catch (e) {
      console.log(e);
    }
  };

  const handleDisconnect = () => {
    try {
      ros.ROS.close();
      setROS((ros) => ({ 
        ...ros, 
        isConnected: false,
        error: null
      }));
    } catch (e) {
      console.log(e);
    }
    console.log("Disconnected from websocket server.");
  };

  const sendServiceRequest = (serviceObj, callback) => {
    try {
      const { action, id, gains } = serviceObj;

      let request;
      if(id === CONTROLLER_TYPE_SERVICE_ID) {
        request = new ServiceRequest({ type: action });
        controllerTypeService.callService(request, callback);
      } else if(id === PID_TUNING_SERVICE_ID) {
        request = new ServiceRequest({ pid: gains });
        pidTuningService.callService(request, callback);
      } else if(id === ACTION_SERVICE_ID) {
        request = new ServiceRequest({ action });
        actionService.callService(request, callback);
      }

    } catch (e) {
      console.log(e, e.message);
    }
  }

  return {
    changeUrl,
    handleConnect,
    handleDisconnect,
    sendServiceRequest,
    ros: ros.ROS,
    isConnected: true,
    error: ros.error,
    url: ros.url,
  };
}

const ROS = (props) => {
  return <ROSProvider>{props.children}</ROSProvider>;
};

export { useROS, ROS };
