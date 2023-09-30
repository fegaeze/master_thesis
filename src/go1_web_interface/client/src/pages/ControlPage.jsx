import React from "react";
import { actions, tabHeaders } from "./config";
import Tab from "../components/Tab";
import Card from "../components/Card";
import Header from "../components/Header";
import QuickAction from "../components/QuickAction";

const ControlPage = () => {
  const [currentAction, setCurrentAction] = React.useState(null);
  const [previousAction, setPreviousAction] = React.useState(null);
  const [controlMethod, setControlMethod] = React.useState(null);
  const [currentFootRaised, setCurrentFootRaised] = React.useState(null);

  const handleQuickActionClick = action => {
    setCurrentAction(action);
  }

  return (
    <>
      <div className="flex min-h-full flex-1 flex-col">
        <Header />
        <main className="lg:max-w-[1000px] mx-auto mb-4 w-full py-6 px-4 sm:px-6 lg:px-8">
          <div className="mt-10 mb-[4rem] my-5 grid grid-cols-1 sm:grid-cols-2 md:grid-cols-3 gap-4">
            <Card state={currentAction} title="Current Action" />
            <Card state={previousAction} title="Previous Action" />
            <Card state={controlMethod} title="Control Method" />
            <Card state={currentFootRaised} title="Current Foot Raised" />
          </div>
          
          <Tab headers={tabHeaders}>
            <div className="m-5 grid grid-cols-1 sm:grid-cols-2 md:grid-cols-3 lg:grid-cols-4 gap-4">
              {
                actions.map((action, index) => {
                  return (
                    <QuickAction key={index} action={() => handleQuickActionClick(action)} title={action} />
                  );
                })
              }
            </div>

            <div>Tab 2</div>
          </Tab>
        </main>
      </div>
    </>
  );
};

export default ControlPage;
