// Create this as a new file: RobotTelemetryServer.java

package frc.robot;

import java.net.DatagramSocket;
import java.net.DatagramPacket;
import java.net.InetAddress;
import java.util.Set;
import java.util.HashSet;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.VisionConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

/**
 * UDP Telemetry Server for Robot
 * 
 * Provides simple UDP-based telemetry broadcasting to control boards,
 * driver stations, and other clients. Much simpler than NetworkTables!
 * 
 * Only sends packets when data changes or on heartbeat interval.
 */
public class RobotTelemetryServer {
    
    private final int teamNumber;
    private DatagramSocket udpSocket;
    private DatagramSocket discoverySocket;
    private boolean serverRunning = false;
    private Thread broadcastThread;
    private Thread discoveryThread;
    private Set<String> knownClients = new HashSet<>();
    
    // Telemetry data suppliers (lambda functions to get current values)
    private Supplier<Double> batteryVoltageSupplier = () -> RobotController.getBatteryVoltage(); // Use RIO voltage
    private Supplier<Boolean> shooterReadySupplier = () -> false;  // Fixed for testing
    private Supplier<Integer> shooterSpeedSupplier = () -> 0;      // Fixed for testing
    private Supplier<Boolean> intakeDeployedSupplier = () -> false; // Fixed for testing
    private Supplier<Double> intakePositionSupplier = () -> 0.0;    // Fixed for testing
    private Supplier<String> autoModeSupplier = () -> "Center";
    private Supplier<Integer> autoModeNumberSupplier = () -> 1;
    private Supplier<String> statusMessageSupplier = () -> "Robot Running";
    private Supplier<Double> matchTimeSupplier = () -> 0.0;
    
    // Network configuration
    private static final int TELEMETRY_PORT = 5574;
    private static final int DISCOVERY_PORT = 5575;
    private static final int CHECK_INTERVAL_MS = 100; // Check for changes every 100ms
    private static final int HEARTBEAT_INTERVAL_MS = 5000; // Force send every 5 seconds
    
    // State tracking for send-on-change
    private TelemetryState previousState = new TelemetryState();
    private long lastForcedSendTime = 0;
    private boolean forceNextSend = false;
    
    public RobotTelemetryServer(int teamNumber) {
        this.teamNumber = teamNumber;
    }
    
    /**
     * Tracks previous telemetry values to detect changes
     */
    private static class TelemetryState {
        double timestamp = -1;
        
        // Battery
        double batteryVoltage = -1;
        boolean batteryIsLow = false;
        
        // Robot state
        boolean robotEnabled = false;
        boolean isAutonomous = false;
        boolean isTeleop = false;
        String robotMode = "";
        
        // Alliance
        String allianceColor = "";
        boolean isRedAlliance = false;
        
        // Match
        double matchTimeRemaining = -1;
        
        // Subsystems
        boolean shooterReady = false;
        int shooterSpeed = -1;
        boolean intakeDeployed = false;
        double intakePosition = -1;

        // Auto Align Status
        boolean leftCameraSeesTag = false;
        boolean rightCameraSeesTag = false;
        double leftCameraTag = -1;
        double rightCameraTag = -1;
        
        // Auto
        String autoMode = "";
        int autoModeNumber = -1;
        
        // Status
        String statusMessage = "";
        int heartbeat = -1;
        
        /**
         * Compare this state with current values and return true if anything changed
         * NOTE: Ignores timestamp and heartbeat since they always change
         */
        boolean hasChanges(double currentTimestamp, double batteryVolt, boolean batteryLow,
                          boolean enabled, boolean auto, boolean teleop, String mode,
                          String alliance, boolean isRed, double matchTime,
                          boolean shootReady, int shootSpeed, boolean intakeDepl, double intakePos,
                          String autoMd, int autoNum, String statusMsg, int hb, boolean lfCamHasTag,
                          boolean rtCamHasTag, double lfTag, double rtTag) {
            
            return Math.abs(batteryVoltage - batteryVolt) > 0.01 ||
                   batteryIsLow != batteryLow ||
                   robotEnabled != enabled ||
                   isAutonomous != auto ||
                   isTeleop != teleop ||
                   !robotMode.equals(mode) ||
                   !allianceColor.equals(alliance) ||
                   isRedAlliance != isRed ||
                   Math.abs(matchTimeRemaining - matchTime) > 0.1 ||
                   shooterReady != shootReady ||
                   shooterSpeed != shootSpeed ||
                   intakeDeployed != intakeDepl ||
                   Math.abs(intakePosition - intakePos) > 0.01 ||
                   !autoMode.equals(autoMd) ||
                   autoModeNumber != autoNum ||
                   !statusMessage.equals(statusMsg) ||
                   leftCameraSeesTag != lfCamHasTag ||
                   rightCameraSeesTag != rtCamHasTag ||
                   Math.abs(leftCameraTag - lfTag) > 0.01 ||
                   Math.abs(rightCameraTag - rtTag) > 0.01;
                   // NOTE: Removed timestamp and heartbeat comparisons
        }
        
        /**
         * Update this state with current values
         * NOTE: Still tracks timestamp and heartbeat for packet generation
         */
        void update(double currentTimestamp, double batteryVolt, boolean batteryLow,
                   boolean enabled, boolean auto, boolean teleop, String mode,
                   String alliance, boolean isRed, double matchTime,
                   boolean shootReady, int shootSpeed, boolean intakeDepl, double intakePos,
                   String autoMd, int autoNum, String statusMsg, int hb, boolean lfCamHasTag, 
                   boolean rtCamHasTag, double lfTag, double rtTag) {
            
            timestamp = currentTimestamp;
            batteryVoltage = batteryVolt;
            batteryIsLow = batteryLow;
            robotEnabled = enabled;
            isAutonomous = auto;
            isTeleop = teleop;
            robotMode = mode;
            allianceColor = alliance;
            isRedAlliance = isRed;
            matchTimeRemaining = matchTime;
            shooterReady = shootReady;
            shooterSpeed = shootSpeed;
            intakeDeployed = intakeDepl;
            intakePosition = intakePos;
            autoMode = autoMd;
            autoModeNumber = autoNum;
            statusMessage = statusMsg;
            heartbeat = hb;
            leftCameraSeesTag = lfCamHasTag;
            rightCameraSeesTag = rtCamHasTag;
            leftCameraTag = lfTag;
            rightCameraTag = rtTag;
        }
    }

    
    /**
     * Start the telemetry server
     */
    public boolean start() {
        try {
            System.out.println("=== Starting Robot Telemetry Server ===");
            System.out.println("Team: " + teamNumber);
            
            // Create sockets
            udpSocket = new DatagramSocket();
            udpSocket.setBroadcast(true);
            discoverySocket = new DatagramSocket(DISCOVERY_PORT);
            
            serverRunning = true;
            
            System.out.println("✅ Telemetry socket created on port: " + udpSocket.getLocalPort());
            System.out.println("✅ Discovery socket listening on port: " + DISCOVERY_PORT);
            
            // Start threads
            broadcastThread = new Thread(this::broadcastLoop, "TelemetryBroadcast");
            broadcastThread.setDaemon(true);
            broadcastThread.start();
            
            discoveryThread = new Thread(this::discoveryLoop, "TelemetryDiscovery");
            discoveryThread.setDaemon(true);
            discoveryThread.start();
            
            System.out.println("✅ Telemetry server started successfully");
            System.out.println("📡 Send-on-change mode: packets sent only when data changes");
            return true;
            
        } catch (Exception e) {
            System.err.println("❌ Failed to start telemetry server: " + e.getMessage());
            stop();
            return false;
        }
    }
    
    /**
     * Stop the telemetry server
     */
    public void stop() {
        serverRunning = false;
        
        if (broadcastThread != null) {
            broadcastThread.interrupt();
        }
        if (discoveryThread != null) {
            discoveryThread.interrupt();
        }
        if (udpSocket != null) {
            udpSocket.close();
        }
        if (discoverySocket != null) {
            discoverySocket.close();
        }
        
        System.out.println("✅ Telemetry server stopped");
    }
    
    /**
     * Force the next telemetry packet to be sent (useful after config changes)
     */
    public void forceSend() {
        forceNextSend = true;
    }
    
    /**
     * Set custom data suppliers for real robot subsystems
     */
    public void setBatteryVoltageSupplier(Supplier<Double> supplier) {
        this.batteryVoltageSupplier = supplier;
    }
    
    public void setShooterSuppliers(Supplier<Boolean> readySupplier, Supplier<Integer> speedSupplier) {
        this.shooterReadySupplier = readySupplier;
        this.shooterSpeedSupplier = speedSupplier;
    }
    
    public void setIntakeSuppliers(Supplier<Boolean> deployedSupplier, Supplier<Double> positionSupplier) {
        this.intakeDeployedSupplier = deployedSupplier;
        this.intakePositionSupplier = positionSupplier;
    }
    
    public void setAutoSuppliers(Supplier<String> modeSupplier, Supplier<Integer> numberSupplier) {
        this.autoModeSupplier = modeSupplier;
        this.autoModeNumberSupplier = numberSupplier;
    }
    
    public void setStatusMessageSupplier(Supplier<String> supplier) {
        this.statusMessageSupplier = supplier;
    }
    
    /**
     * Get server status
     */
    public boolean isRunning() {
        return serverRunning;
    }
    
    public int getClientCount() {
        return knownClients.size();
    }
    
    public Set<String> getKnownClients() {
        return new HashSet<>(knownClients); // Return copy
    }
    
    // Private methods
    
    private void discoveryLoop() {
        System.out.println("🔍 Discovery listener started");
        byte[] buffer = new byte[512];
        
        while (serverRunning) {
            try {
                DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                discoverySocket.receive(packet);
                
                String message = new String(packet.getData(), 0, packet.getLength());
                String clientIP = packet.getAddress().getHostAddress();
                
                System.out.println("📡 Discovery from " + clientIP + ": " + message);
                
                // Simple parsing - look for teensy control board
                if (message.contains("teensy_control_board") || message.contains("control_board")) {
                    boolean isNewClient = knownClients.add(clientIP);
                    if (isNewClient) {
                        System.out.println("✅ Registered new client: " + clientIP);
                    }
                    
                    // Force send telemetry to new or existing client
                    System.out.println("🚀 Forcing telemetry send after discovery");
                    forceNextSend = true;
                }
                
            } catch (Exception e) {
                if (serverRunning) {
                    System.err.println("Discovery error: " + e.getMessage());
                }
            }
        }
        
        System.out.println("🛑 Discovery listener stopped");
    }
    
    private void broadcastLoop() {
        System.out.println("🚀 Telemetry broadcast started (send-on-change mode)");
        
        while (serverRunning) {
            try {
                long currentTime = System.currentTimeMillis();
                boolean shouldSendHeartbeat = (currentTime - lastForcedSendTime) >= HEARTBEAT_INTERVAL_MS;
                
                // Get current telemetry values
                double currentTimestamp = Timer.getFPGATimestamp();
                double batteryVoltage = batteryVoltageSupplier.get();
                boolean batteryIsLow = batteryVoltage < 11.5;
                boolean robotEnabled = DriverStation.isEnabled();
                boolean isAutonomous = DriverStation.isAutonomous();
                boolean isTeleop = DriverStation.isTeleop();
                String robotMode = getRobotModeString();
                
                String allianceColor = "unknown";
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    allianceColor = alliance.get().toString().toLowerCase();
                }
                boolean isRedAlliance = allianceColor.equals("red");
                
                double matchTimeRemaining = DriverStation.getMatchTime();
                boolean shooterReady = shooterReadySupplier.get();
                int shooterSpeed = shooterSpeedSupplier.get();
                boolean intakeDeployed = intakeDeployedSupplier.get();
                double intakePosition = intakePositionSupplier.get();
                String autoMode = autoModeSupplier.get();
                int autoModeNumber = autoModeNumberSupplier.get();
                String statusMessage = statusMessageSupplier.get();
                int heartbeat = (int)(Timer.getFPGATimestamp() * 10) % 1000;

                boolean leftCamHasTag = LimelightHelpers.getTV(VisionConstants.leftCameraName);
                boolean rightCamHasTag = LimelightHelpers.getTV(VisionConstants.rightCameraName);
                double leftCamTag = LimelightHelpers.getFiducialID(VisionConstants.leftCameraName);
                double rightCamTag = LimelightHelpers.getFiducialID(VisionConstants.rightCameraName);
                
                // Check if we should send
                boolean hasChanges = previousState.hasChanges(
                    currentTimestamp, batteryVoltage, batteryIsLow,
                    robotEnabled, isAutonomous, isTeleop, robotMode,
                    allianceColor, isRedAlliance, matchTimeRemaining,
                    shooterReady, shooterSpeed, intakeDeployed, intakePosition,
                    autoMode, autoModeNumber, statusMessage, heartbeat, 
                    leftCamHasTag, rightCamHasTag, leftCamTag, rightCamTag
                );
                
                boolean shouldSend = hasChanges || forceNextSend || shouldSendHeartbeat;
                
                if (shouldSend) {
                    String telemetryJson = createTelemetryPacket(
                        currentTimestamp, batteryVoltage, batteryIsLow,
                        robotEnabled, isAutonomous, isTeleop, robotMode,
                        allianceColor, isRedAlliance, matchTimeRemaining,
                        shooterReady, shooterSpeed, intakeDeployed, intakePosition,
                        autoMode, autoModeNumber, statusMessage, heartbeat,
                        leftCamHasTag, rightCamHasTag, leftCamTag, rightCamTag
                    );
                    
                    byte[] data = telemetryJson.getBytes();
                    int packetsSent = 0;
                    
                    // Send to all known clients
                    for (String clientIP : knownClients) {
                        try {
                            InetAddress clientAddr = InetAddress.getByName(clientIP);
                            DatagramPacket packet = new DatagramPacket(data, data.length, clientAddr, TELEMETRY_PORT);
                            udpSocket.send(packet);
                            packetsSent++;
                        } catch (Exception e) {
                            System.err.println("Failed to send to " + clientIP + ": " + e.getMessage());
                        }
                    }
                    
                    // Also broadcast to subnet for discovery
                    try {
                        String broadcastIP = calculateBroadcastIP();
                        InetAddress broadcastAddr = InetAddress.getByName(broadcastIP);
                        DatagramPacket packet = new DatagramPacket(data, data.length, broadcastAddr, TELEMETRY_PORT);
                        udpSocket.send(packet);
                        packetsSent++;
                    } catch (Exception e) {
                        System.err.println("Broadcast failed: " + e.getMessage());
                    }
                    
                    // Update state tracking
                    previousState.update(
                        currentTimestamp, batteryVoltage, batteryIsLow,
                        robotEnabled, isAutonomous, isTeleop, robotMode,
                        allianceColor, isRedAlliance, matchTimeRemaining,
                        shooterReady, shooterSpeed, intakeDeployed, intakePosition,
                        autoMode, autoModeNumber, statusMessage, heartbeat,
                        leftCamHasTag, rightCamHasTag, leftCamTag, rightCamTag
                    );
                    
                    String sendReason = forceNextSend ? "FORCED" : 
                                      shouldSendHeartbeat ? "HEARTBEAT" : "CHANGE";
                    
                    //System.out.println("📤 Sent telemetry (" + sendReason + ") to " + packetsSent + " destinations");
                    
                    if (shouldSendHeartbeat) {
                        lastForcedSendTime = currentTime;
                    }
                    forceNextSend = false;
                }
                
                Thread.sleep(CHECK_INTERVAL_MS);
                
            } catch (InterruptedException e) {
                break;
            } catch (Exception e) {
                System.err.println("Broadcast error: " + e.getMessage());
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException ie) {
                    break;
                }
            }
        }
        
        System.out.println("🛑 Telemetry broadcast stopped");
    }
    
    private String createTelemetryPacket(double currentTimestamp, double batteryVoltage, boolean batteryIsLow,
                                       boolean robotEnabled, boolean isAutonomous, boolean isTeleop, String robotMode,
                                       String allianceColor, boolean isRedAlliance, double matchTimeRemaining,
                                       boolean shooterReady, int shooterSpeed, boolean intakeDeployed, double intakePosition,
                                       String autoMode, int autoModeNumber, String statusMessage, int heartbeat, boolean lfCamHasTag,
                                       boolean rtCamHasTag, double lfTag, double rtTag) {
        StringBuilder json = new StringBuilder();
        json.append("{");
        
        // Metadata
        json.append("\"timestamp\":").append(currentTimestamp).append(",");
        json.append("\"team\":").append(teamNumber).append(",");
        
        // Battery
        json.append("\"battery\":{");
        json.append("\"voltage\":").append(batteryVoltage).append(",");
        json.append("\"isLow\":").append(batteryIsLow);
        json.append("},");
        
        // Robot state
        json.append("\"robot\":{");
        json.append("\"enabled\":").append(robotEnabled).append(",");
        json.append("\"autonomous\":").append(isAutonomous).append(",");
        json.append("\"teleop\":").append(isTeleop).append(",");
        json.append("\"mode\":\"").append(robotMode).append("\"");
        json.append("},");
        
        // Alliance
        json.append("\"alliance\":{");
        json.append("\"color\":\"").append(allianceColor).append("\",");
        json.append("\"isRed\":").append(isRedAlliance);
        json.append("},");
        
        // Match
        json.append("\"match\":{");
        json.append("\"timeRemaining\":").append(matchTimeRemaining);
        json.append("},");
        
        // Subsystems (using suppliers)
        json.append("\"shooter\":{");
        json.append("\"ready\":").append(shooterReady).append(",");
        json.append("\"speed\":").append(shooterSpeed);
        json.append("},");
        
        json.append("\"intake\":{");
        json.append("\"deployed\":").append(intakeDeployed).append(",");
        json.append("\"position\":").append(intakePosition);
        json.append("},");

        // Auto Align Status
        json.append("\"vision\":{");
            json.append("\"leftCam\":{");
                json.append("\"hasTag\":").append(lfCamHasTag).append(",");
                json.append("\"tagID\":").append(lfTag);
            json.append("},");

            json.append("\"rightCam\":{");
                json.append("\"hasTag\":").append(rtCamHasTag).append(",");
                json.append("\"tagID\":").append(rtTag);
            json.append("}");
        json.append("},");
            
        // Auto
        json.append("\"auto\":{");
        json.append("\"selectedMode\":\"").append(autoMode).append("\",");
        json.append("\"modeNumber\":").append(autoModeNumber);
        json.append("},");
        
        // Status
        json.append("\"status\":{");
        json.append("\"message\":\"").append(statusMessage).append("\",");
        json.append("\"heartbeat\":").append(heartbeat);
        json.append("}");
        
        json.append("}");
        return json.toString();
    }
    
    private String getRobotModeString() {
        if (DriverStation.isDisabled()) return "Disabled";
        if (DriverStation.isAutonomous()) return "Autonomous";
        if (DriverStation.isTeleop()) return "Teleop";
        if (DriverStation.isTest()) return "Test";
        return "Unknown";
    }
    
    private String calculateBroadcastIP() {
        // Calculate broadcast IP from team number: 10.TE.AM.255
        int firstOctet = teamNumber / 100;
        int secondOctet = teamNumber % 100;
        return "10." + firstOctet + "." + secondOctet + ".255";
    }
}