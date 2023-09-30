import React, { useContext } from "react";
import { ROSContext, ROSProvider } from "../context/ROSContext";

function useROS() {
  const [ros, setROS] = useContext(ROSContext);

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
        error: null,
        topics: [],
        listeners: []
      }));
    } catch (e) {
      console.log(e);
    }
    console.log("Disconnected from websocket server.");
  };

  return {
    changeUrl,
    handleConnect,
    handleDisconnect,
    ros: ros.ROS,
    isConnected: ros.isConnected,
    error: ros.error,
    url: ros.url,
    topics: ros.topics,
    services: ros.services,
    listeners: ros.listeners,
  };
}

const ROS = (props) => {
  return <ROSProvider>{props.children}</ROSProvider>;
};

export { useROS, ROS };
