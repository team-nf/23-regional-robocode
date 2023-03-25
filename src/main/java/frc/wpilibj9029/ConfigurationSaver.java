package frc.wpilibj9029;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Iterator;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.json.JSONException;
import org.json.JSONObject;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ConfigurationSaver {
    private static JSONObject config = new JSONObject();
    static {
        config.put("DriveBase", new JSONObject());
        config.put("Turret", new JSONObject());
        config.put("Lift", new JSONObject());
        config.put("Arm", new JSONObject());
        config.put("Wrist", new JSONObject());
    }

    public static void save(String filepath) {
        for (Iterator<String> i = SmartDashboard.getKeys().iterator(); i.hasNext(); i.next()) {
            if(i.next().split("/")[0] == "DriveBase") {config.getJSONObject("DriveBase").put(i.next(), SmartDashboard.getData(i.next()));}
            if(i.next().split("/")[0] == "Turret") {config.getJSONObject("Turret").put(i.next(), SmartDashboard.getData(i.next()));}
            if(i.next().split("/")[0] == "Lift") {config.getJSONObject("Lift").put(i.next(), SmartDashboard.getData(i.next()));}
            if(i.next().split("/")[0] == "Arm") {config.getJSONObject("Arm").put(i.next(), SmartDashboard.getData(i.next()));}
            if(i.next().split("/")[0] == "Wrist") {config.getJSONObject("Wrist").put(i.next(), SmartDashboard.getData(i.next()));}
        }
        
        try {
        FileWriter file = new FileWriter(filepath);
        file.write(config.toString());
        file.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        System.out.println(String.format("json file saved at %s", filepath));
    }

    public static void load(String filepath) {
        Path path = Path.of(filepath);
        try {
            Stream<String> jfile = Files.lines(path);
            String jstr = jfile.collect(Collectors.joining());
            jfile.close();
            config = new JSONObject(jstr);
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    public static JSONObject getJSON() {return config;}

    public static double getParameter(String param, double defaultValue) {
        String cls = param.split("/")[0];
        String parm = param.split("/")[param.length() - 1];
        try {
        return config.getJSONObject(cls).getDouble(parm);
        } catch (JSONException j) {
            return defaultValue;
        }
    }

    public static HashMap<String, Double> getCoeffs(String subsystem, HashMap<String, Double> defaultValue) {
        JSONObject jcoeffs = config.getJSONObject(subsystem);
        HashMap<String, Double> coeffs = new HashMap<String, Double>();
        for (Iterator<String> i = config.getJSONObject(subsystem).keys(); i.hasNext(); i.next()) {
            double val;
            try {val = jcoeffs.getDouble(i.next());}
            catch (JSONException e) {val = defaultValue.get(i.next());}
            coeffs.put(i.next(), val);
        }
        return coeffs;
    }
}
