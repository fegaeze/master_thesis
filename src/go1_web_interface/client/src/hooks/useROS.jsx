import React, { useContext } from "react";
import { ServiceRequest } from "roslib";
import { ROSContext, ROSProvider } from "../context/ROSContext";

function useROS() {
  const [ros, actionService, controllerService, setROS] = useContext(ROSContext);

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
      const { action, id } = serviceObj;
      const request = new ServiceRequest({ action });
      if(id === "CONTROLLER") {
        controllerService.callService(request, callback);
      } else {
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
    isConnected: ros.isConnected,
    error: ros.error,
    url: ros.url,
  };
}

const ROS = (props) => {
  return <ROSProvider>{props.children}</ROSProvider>;
};

export { useROS, ROS };
