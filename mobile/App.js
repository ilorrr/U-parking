import React from "react";
import { NavigationContainer } from "@react-navigation/native";
import { createNativeStackNavigator } from "@react-navigation/native-stack";


import HomeScreen from "./src/screens/HomeScreen";
import Dashboard from "./src/screens/Dashboard";


const Stack = createNativeStackNavigator();


function LotScreen() {
 return null;
}


export default function App() {
 return (
   <NavigationContainer>
     <Stack.Navigator>


       <Stack.Screen name="Home" component={HomeScreen} />
       <Stack.Screen name="Dashboard" component={Dashboard} />
       {/*<Stack.Screen name="Lot" component={LotScreen} />*/}


     </Stack.Navigator>
   </NavigationContainer>
 );
}
