import React from "react";
import { useROS } from "../hooks/useROS";

const ControlPage = () => {
  const { handleConnect, handleDisconnect, isConnected } = useROS();

  const handleClick = () => {
    if(isConnected) handleDisconnect();
    else handleConnect();
  }

  return (
    <>
      <div className="flex min-h-full -mt-2 flex-1 flex-col">
        <header className="bg-white border min-[400px]:flex justify-between items-center gap-x-6 py-6 px-4 sm:px-6 lg:px-8">
          <div className="min-[570px]:inline-flex items-baseline">
            <h1 className="text-3xl font-light tracking-tight text-gray-900">Control Dashboard</h1>

            <div className="flex items-center gap-x-1.5 min-[570px]:px-4">
              <div className={`flex-none rounded-full ${isConnected ? "bg-emerald-500/20" : "bg-red-500/20"} p-1`}>
                <div className={`h-1.5 w-1.5 rounded-full ${isConnected ? "bg-emerald-500" : "bg-red-500"}`} />
              </div>
              <p className="text-xs leading-5 text-gray-500">{isConnected ? "Rosbridge Connected" : "Rosbridge Disonnected"}</p>
            </div>
          </div>
          <button
            onClick={handleClick}
            className="inline-flex max-[400px]:w-full justify-center mt-3 min-[400px]:mt-0 items-center rounded-md bg-indigo-600 px-3 py-2 text-sm font-semibold text-white shadow-sm hover:bg-indigo-500 focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-indigo-600"
          >
            {isConnected ? "Disconnect" : "Connect"}
          </button>
          
        </header>
        <main>
          <div className="mx-auto max-w-7xl py-6 sm:px-6 lg:px-8">{/* Your content */}</div>
        </main>
      </div>
    </>
  );
};

export default ControlPage;
