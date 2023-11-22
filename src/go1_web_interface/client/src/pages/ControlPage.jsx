import React, { useEffect, useState } from "react";
import Header from "../components/Header";
import CallToActionBtn from "../components/CallToActionBtn";
import { useROS } from "../hooks/useROS";
import { PID_TUNING_SERVICE_ID, ctas } from "../config";


const initialInputGains = {
  kp_push: '',
  ki_push: '',
  kd_push: '',
  kp_pull: '',
  ki_pull: '',
  kd_pull: ''
}

const initialGains = {
  kp_push: 0.0,
  ki_push: 0.0,
  kd_push: 0.0,
  kp_pull: 0.0,
  ki_pull: 0.0,
  kd_pull: 0.0
}

const ControlPage = () => {
  const { sendServiceRequest } = useROS();

  const [servicePayload, setServicePayload] = useState({
    id: null,
    action: null,
    gains: null
  });

  const [currentGains, setCurrentGains] = useState(initialGains);
  const [pidGains, setPIDGains] = useState(initialInputGains);

  useEffect(() => {
    sendServiceRequest(servicePayload, res => {
      console.log("Service Request Response", res);
    });
  }, [setServicePayload])

  const handleCallToActionBtnClick = (actionKey, actionID) => {
    setServicePayload({
      ...servicePayload,
      action: actionKey,
      id: actionID
    });
  }

  const handlePIDTuningFormSubmit = evt => {
    evt.preventDefault();
    
    const gains = {
      kp_push: pidGains["kp_push"] === '' ? currentGains["kp_push"]: Number(pidGains["kp_push"]),
      ki_push: pidGains["ki_push"] === '' ? currentGains["ki_push"]: Number(pidGains["ki_push"]),
      kd_push: pidGains["kd_push"] === '' ? currentGains["kd_push"]: Number(pidGains["kd_push"]),
      kp_pull: pidGains["kp_pull"] === '' ? currentGains["kp_pull"]: Number(pidGains["kp_pull"]),
      ki_pull: pidGains["ki_pull"] === '' ? currentGains["ki_pull"]: Number(pidGains["ki_pull"]),
      kd_pull: pidGains["kd_pull"] === '' ? currentGains["kd_pull"]: Number(pidGains["kd_pull"])
    }

    setServicePayload({
      ...servicePayload,
      gains, 
      id: PID_TUNING_SERVICE_ID
    });

    setCurrentGains(gains);
    setPIDGains(initialInputGains);
  }

  const handlePIDChange = (id, val) => {
    setPIDGains({
      ...pidGains,
      [id]: val
    });
  }

  return (
    <>
      <div className="flex min-h-full flex-1 flex-col">
        <Header />
        <main className="lg:max-w-[1000px] mx-auto mb-4 w-full py-6 px-4 sm:px-6 lg:px-8">
          <div className="my-8 grid grid-cols-1 sm:grid-cols-2 gap-8">
            <div className="border rounded-lg">
              <div className="flex rounded-t-lg px-6 py-3 bg-white text-gray-600">
                <h2>PID Tuning</h2>
              </div> 
              <div className="px-6 py-4">
                <form onSubmit={(evt) => handlePIDTuningFormSubmit(evt)}>
                  <div className="mb-6">
                    <label htmlFor="kp_push" 
                      className="block mb-2 text-sm font-medium text-gray-900 dark:text-white"
                    >P Gain - Push: {currentGains["kp_push"]}</label>
                    <input 
                      type="number"
                      id="kp_push" 
                      step="any"
                      value={pidGains["kp_push"]}
                      onChange={evt => handlePIDChange("kp_push", evt.target.value)} 
                      className="bg-gray-50 border border-gray-300 text-gray-900 text-sm rounded-lg focus:ring-indigo-500 focus:border-indigo-500 block w-full p-2.5 dark:bg-gray-700 dark:border-gray-600 dark:placeholder-gray-400 dark:text-white dark:focus:ring-indigo-500 dark:focus:border-indigo-500" />
                  </div>

                  <div className="mb-6">
                    <label htmlFor="kp_pull" 
                      className="block mb-2 text-sm font-medium text-gray-900 dark:text-white"
                    >P Gain - Pull: {currentGains["kp_pull"]}</label>
                    <input 
                      type="number"
                      id="kp_pull" 
                      step="any"
                      value={pidGains["kp_pull"]}
                      onChange={evt => handlePIDChange("kp_pull", evt.target.value)} 
                      className="bg-gray-50 border border-gray-300 text-gray-900 text-sm rounded-lg focus:ring-indigo-500 focus:border-indigo-500 block w-full p-2.5 dark:bg-gray-700 dark:border-gray-600 dark:placeholder-gray-400 dark:text-white dark:focus:ring-indigo-500 dark:focus:border-indigo-500" />
                  </div>

                  <div className="mb-6">
                    <label htmlFor="ki_push" 
                      className="block mb-2 text-sm font-medium text-gray-900 dark:text-white"
                    >I Gain - Push: {currentGains["ki_push"]}</label>
                    <input 
                      type="number"
                      id="ki_push" 
                      step="any"
                      value={pidGains["ki_push"]}
                      onChange={evt => handlePIDChange("ki_push", evt.target.value)} 
                      className="bg-gray-50 border border-gray-300 text-gray-900 text-sm rounded-lg focus:ring-indigo-500 focus:border-indigo-500 block w-full p-2.5 dark:bg-gray-700 dark:border-gray-600 dark:placeholder-gray-400 dark:text-white dark:focus:ring-indigo-500 dark:focus:border-indigo-500" />
                  </div>

                  <div className="mb-6">
                    <label htmlFor="ki_pull" 
                      className="block mb-2 text-sm font-medium text-gray-900 dark:text-white"
                    >I Gain - Pull: {currentGains["ki_pull"]}</label>
                    <input 
                      type="number"
                      id="ki_pull" 
                      step="any"
                      value={pidGains["ki_pull"]}
                      onChange={evt => handlePIDChange("ki_pull", evt.target.value)} 
                      className="bg-gray-50 border border-gray-300 text-gray-900 text-sm rounded-lg focus:ring-indigo-500 focus:border-indigo-500 block w-full p-2.5 dark:bg-gray-700 dark:border-gray-600 dark:placeholder-gray-400 dark:text-white dark:focus:ring-indigo-500 dark:focus:border-indigo-500" />
                  </div>

                  <div className="mb-6">
                    <label htmlFor="kd_push" 
                      className="block mb-2 text-sm font-medium text-gray-900 dark:text-white"
                    >D Gain - Push: {currentGains["kd_push"]}</label>
                    <input 
                      type="number"
                      id="kd_push" 
                      step="any"
                      value={pidGains["kd_push"]}
                      onChange={evt => handlePIDChange("kd_push", evt.target.value)} 
                      className="bg-gray-50 border border-gray-300 text-gray-900 text-sm rounded-lg focus:ring-indigo-500 focus:border-indigo-500 block w-full p-2.5 dark:bg-gray-700 dark:border-gray-600 dark:placeholder-gray-400 dark:text-white dark:focus:ring-indigo-500 dark:focus:border-indigo-500" />
                  </div>

                  <div className="mb-6">
                    <label htmlFor="kd_pull" 
                      className="block mb-2 text-sm font-medium text-gray-900 dark:text-white"
                    >D Gain - Pull: {currentGains["kd_pull"]}</label>
                    <input 
                      type="number"
                      id="kd_pull" 
                      step="any"
                      value={pidGains["kd_pull"]}
                      onChange={evt => handlePIDChange("kd_pull", evt.target.value)} 
                      className="bg-gray-50 border border-gray-300 text-gray-900 text-sm rounded-lg focus:ring-indigo-500 focus:border-indigo-500 block w-full p-2.5 dark:bg-gray-700 dark:border-gray-600 dark:placeholder-gray-400 dark:text-white dark:focus:ring-indigo-500 dark:focus:border-indigo-500" />
                  </div>

                  <button 
                    type="submit" 
                    className="text-white bg-indigo-700 hover:bg-indigo-800 focus:ring-4 focus:outline-none focus:ring-indigo-300 font-medium rounded-lg text-sm w-full sm:w-auto px-5 py-2.5 text-center dark:bg-indigo-600 dark:hover:bg-indigo-700 dark:focus:ring-indigo-800"
                  >Submit</button>
                </form>
              </div>
            </div>

            <div className="border rounded-lg">
              <div className="flex rounded-t-lg px-6 py-3 bg-white text-gray-600">
                <h2>FIS Tuning</h2>
              </div> 
              <div className="p-4">
                <div className="m-5 grid grid-cols-1 sm:grid-cols-2 md:grid-cols-3 lg:grid-cols-4 gap-4">
                  Hello
                </div>
              </div>
            </div>
          </div>
          {
            ctas.map(({ id, title, actions }) => (
              <div key={id} className="border rounded-lg mb-8">
                <div className="flex rounded-t-lg px-6 py-3 bg-white text-gray-600">
                  <h2>{title}</h2>
                </div> 
                <div className="p-4">
                  <div className="m-5 grid grid-cols-1 sm:grid-cols-2 md:grid-cols-3 lg:grid-cols-4 gap-4">
                    {
                      actions.map(({ key, title }) => {
                        return (
                          <CallToActionBtn key={key} action={() => handleCallToActionBtnClick(key, id)} title={title} />
                        );
                      })
                    }
                  </div>
                </div>
              </div>
            ))
          }
        </main>
      </div>
    </>
  );
};

export default ControlPage;
