package org.firstinspires.ftc.teamcode.ftc10515.subsystems;

import org.firstinspires.ftc.teamcode.ftc10515.Robot;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

public class ExpansionHubs implements Subsystem {
    private static ExpansionHubs instance;

    public static ExpansionHubs getInstance(ExpansionHubEx masterHub, ExpansionHubEx slaveHub) {
        if(instance == null) {
            instance = new ExpansionHubs(masterHub, slaveHub);
        }

        return instance;
    }

    private ExpansionHubEx masterHub;
    private ExpansionHubEx slaveHub;
    private RevBulkData masterData;
    private RevBulkData slaveData;

    public ExpansionHubs(ExpansionHubEx masterHub, ExpansionHubEx slaveHub) {
        setMasterHub(masterHub);
        setSlaveHub(slaveHub);
    }

    @Override
    public void start() {
        getRevBultData();
    }

    @Override
    public void update(double dt) {
        getRevBultData(dt);
    }

    @Override
    public void stop() {

    }

    public void getRevBultData() {
        final RevBulkData masterData;
        try {
            masterData = getMasterHub().getBulkInputData();
            if(masterData != null) {
                setMasterData(masterData);
            }
        } catch(Exception e) {
            //
        }

        final RevBulkData slaveData;
        try {
            slaveData = getSlaveHub().getBulkInputData();
            if(slaveData != null) {
                setSlaveData(slaveData);
            }
        } catch(Exception e) {
            //
        }

        for(final RevMotor revMotor : Robot.getMotors()) {
            if(revMotor != null) {
                if(revMotor.isOnMasterHub()) {
                    revMotor.setEncoderReading(getMasterData().getMotorCurrentPosition(revMotor.getMotor()));
                } else {
                    revMotor.setEncoderReading(getSlaveData().getMotorCurrentPosition(revMotor.getMotor()));
                }
            }
        }
    }

    public void getRevBultData(double dt) {
        final RevBulkData masterData;
        try {
            masterData = getMasterHub().getBulkInputData();
            if(masterData != null) {
                setMasterData(masterData);
            }
        } catch(Exception e) {
            //
        }

        final RevBulkData slaveData;
        try {
            slaveData = getSlaveHub().getBulkInputData();
            if(slaveData != null) {
                setSlaveData(slaveData);
            }
        } catch(Exception e) {
            //
        }

        for(final RevMotor revMotor : Robot.getMotors()) {
            if(revMotor != null) {
                if(revMotor.isOnMasterHub()) {
                    revMotor.setEncoderReading(getMasterData().getMotorCurrentPosition(revMotor.getMotor()), dt);
                } else {
                    revMotor.setEncoderReading(getSlaveData().getMotorCurrentPosition(revMotor.getMotor()), dt);
                }
            }
        }
    }

    public ExpansionHubEx getMasterHub() {
        return masterHub;
    }

    public void setMasterHub(ExpansionHubEx masterHub) {
        this.masterHub = masterHub;
    }

    public ExpansionHubEx getSlaveHub() {
        return slaveHub;
    }

    public void setSlaveHub(ExpansionHubEx slaveHub) {
        this.slaveHub = slaveHub;
    }

    public RevBulkData getMasterData() {
        return masterData;
    }

    public void setMasterData(RevBulkData masterData) {
        this.masterData = masterData;
    }

    public RevBulkData getSlaveData() {
        return slaveData;
    }

    public void setSlaveData(RevBulkData slaveData) {
        this.slaveData = slaveData;
    }
}
