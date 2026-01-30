import { View, Text } from 'react-native';
import { globalStyles } from './src/styles/globalStyles';

export default function App() {
  return (
    <View style={globalStyles.center}>
      <Text style={globalStyles.title}>Expo test</Text>
    </View>
  );
}
