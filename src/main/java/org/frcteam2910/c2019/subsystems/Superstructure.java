package org.frcteam2910.c2019.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.common.Logger;
import org.frcteam2910.common.robot.drivers.NavX;

import java.io.IOException;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;

public class Superstructure {
    private static final Logger LOGGER = new Logger(Superstructure.class);

    private static final byte[] COMPETITION_BOT_MAC_ADDRESS = new byte[] {
            0x00, (byte) 0x80, 0x2f, 0x24, 0x19, (byte) 0xc0
    };
    private static final byte[] PRACTICE_BOT_MAC_ADDRESS = new byte[] {
            0x00, (byte) 0x80, 0x2f, 0x17, (byte) 0xe5, 0x18
    };

    private static final Superstructure instance = new Superstructure();

    private NavX navX = new NavX(SPI.Port.kMXP);

    private boolean competitionBot;
    private boolean practiceBot;

    private Superstructure() {
        navX.calibrate();
        navX.setInverted(true);

        List<byte[]> macAddresses;
        try {
            macAddresses = getMacAddresses();
        } catch (IOException e) {
            // Don't crash, just log the stacktrace and continue without any mac addresses.
            LOGGER.error(e);
            macAddresses = new ArrayList<>();
        }

        for (byte[] macAddress : macAddresses) {
            // First check if we are the competition bot
            if (Arrays.compare(COMPETITION_BOT_MAC_ADDRESS, macAddress) == 0) {
                competitionBot = true;
                break;
            }

            // Next check if we are the practice bot
            if (Arrays.compare(PRACTICE_BOT_MAC_ADDRESS, macAddress) == 0) {
                practiceBot = true;
                break;
            }
        }

        if (!competitionBot && !practiceBot) {
            String[] macAddressStrings = macAddresses.stream()
                    .map(Superstructure::macToString)
                    .toArray(String[]::new);

            SmartDashboard.putStringArray("MAC Addresses", macAddressStrings);
            SmartDashboard.putString("Competition Bot MAC Address", macToString(COMPETITION_BOT_MAC_ADDRESS));
            SmartDashboard.putString("Practice Bot MAC Address", macToString(PRACTICE_BOT_MAC_ADDRESS));

            // If something goes terribly wrong we still want to use the competition bot stuff in competition.
            competitionBot = true;
        }
    }

    /**
     * Gets the MAC addresses of all present network adapters.
     *
     * @return the MAC addresses of all network adapters.
     */
    private static List<byte[]> getMacAddresses() throws IOException {
        List<byte[]> macAddresses = new ArrayList<>();

        Enumeration<NetworkInterface> networkInterfaces = NetworkInterface.getNetworkInterfaces();

        NetworkInterface networkInterface;
        while (networkInterfaces.hasMoreElements()) {
            networkInterface = networkInterfaces.nextElement();

            byte[] address = networkInterface.getHardwareAddress();
            if (address == null) {
                continue;
            }

            macAddresses.add(address);
        }

        return macAddresses;
    }

    private static String macToString(byte[] address) {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < address.length; i++) {
            if (i != 0) {
                builder.append(':');
            }
            builder.append(String.format("%02X", address[i]));
        }
        return builder.toString();
    }

    public static Superstructure getInstance() {
        return instance;
    }

    public NavX getGyroscope() {
        return navX;
    }

    public boolean isCompetitionBot() {
        return competitionBot;
    }

    public boolean isPracticeBot() {
        return practiceBot;
    }
}
