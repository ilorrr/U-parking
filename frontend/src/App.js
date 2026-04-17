import { Routes, Route } from "react-router-dom";
import Layout from "./components/Layout";
import Home from "./pages/Home";
import Dashboard from "./pages/Dashboard";
import Lot from "./pages/Lot";
import React from "react";

/*IMPORT THE FILE LOCATION HERE*/


/* THIS IS FOR IMPORTING NEW PAGES TO THE WEBSITE AND ADDING TO THE SIDE BAR */


/*IMPORT THE PATH HERE , MAKE SURE PATH= IS CORRECT ALONG WITH ELEMENT= */
export default function App() {
  return (
    <Layout>
      <Routes>
        
        <Route path="/" element={<Home />} />
        <Route path="/dashboard" element={<Dashboard/>} />
        
        <Route path="/lot" element={<Lot />} />
        
        <Route
          path="*"
          element={
            <div className="card">
              <h2>Not found</h2>
            </div>
          }
        />
      </Routes>
    </Layout>
  );
}