import React from "react";
import Header from "../components/Header";
import CallToActionBtn from "../components/CallToActionBtn";
import { useROS } from "../hooks/useROS";
import { ctas } from "../config";

const ControlPage = () => {
  const { sendServiceRequest } = useROS();

  const handleCallToActionBtnClick = (actionKey, actionID ) => {
    sendServiceRequest({ action: actionKey, id: actionID  }, res => {
      console.log("inside sendServiceRequest callback", res);
    });
  }

  return (
    <>
      <div className="flex min-h-full flex-1 flex-col">
        <Header />
        <main className="lg:max-w-[1000px] mx-auto mb-4 w-full py-6 px-4 sm:px-6 lg:px-8">
          {
            ctas.map(({ id, title, actions }) => (
              <div key={id} className="border rounded-lg mb-8">
                <div className="flex rounded-t-lg px-6 py-3 bg-white text-gray-600">
                  <h2 className="">{title}</h2>
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
