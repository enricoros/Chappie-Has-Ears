package com.enricoros.chappiehasears;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.util.UUID;
import java.util.logging.Handler;
import java.util.logging.LogRecord;

public class BluetoothLink implements IControllerOutput {

    private final static String BT_SERIAL_BOARD_UUID = "00001101-0000-1000-8000-00805f9b34fb";

    private final Context mContext;
    private final String mBtTargetDeviceName;
    private final BluetoothAdapter mBluetoothAdapter;
    private final boolean mIsSupported;

    private IntentFilter mBtIntentFilter;
    private boolean mBtReceiverRegistered;
    private BluetoothDevice mBluetoothRemoteDevice;

    private boolean mIsConnected;
    private ConnectThread mConnectThread;
    private ConnectedThread mConnectedThread;

    private static final int COMM_PACKET_SIZE = 5;
    private final byte[] mSendBuffer;

    private Listener mListener;
    interface Listener {
        void btConnectionChanged(boolean connected);
    }

    public BluetoothLink(Context context, String btDeviceName) {
        mSendBuffer = new byte[COMM_PACKET_SIZE];
        mContext = context;
        mBtTargetDeviceName = btDeviceName;

        // check bluetooth_availability and enablement
        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        mIsSupported = mBluetoothAdapter != null || mBluetoothAdapter.isEnabled();
        if (!mIsSupported) {
            Logger.userVisibleMessage("Bluetooth can not be used. Please enable it and restart this APP.");
            //Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            //startActivityForResult(enableBtIntent, Activity.REQUEST_ENABLE_BT);
            return;
        }

        // active and enabled
        mBtIntentFilter = new IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED);
        mBtIntentFilter.addAction(BluetoothDevice.ACTION_FOUND);
        mBtIntentFilter.addAction(BluetoothDevice.ACTION_BOND_STATE_CHANGED);
        mBtIntentFilter.addAction(BluetoothDevice.ACTION_ACL_CONNECTED);
        mBtIntentFilter.addAction(BluetoothDevice.ACTION_ACL_DISCONNECTED);
        mContext.registerReceiver(mReceiver, mBtIntentFilter);
        mBtReceiverRegistered = true;

        // get the device (it may be connected already)
        mBluetoothRemoteDevice = findBondedDeviceByName(mBtTargetDeviceName);
        if (mBluetoothRemoteDevice != null) {
            Logger.userVisibleMessage("Connecting to " + mBtTargetDeviceName);
            connectToRemoteDevice();
        } else {
            Logger.info("BluetoothLink: no bluetooth device paired at startup, starting discovery");
            Logger.userVisibleMessage("Starting discovery for " + mBtTargetDeviceName);
            mBluetoothAdapter.startDiscovery();
        }
    }

    public void setListener(Listener listener) {
        mListener = listener;
        if (mListener != null)
            mListener.btConnectionChanged(mIsConnected);
    }

    public boolean getIsSupported() {
        return mIsSupported;
    }

    public void writeSerialComm(String content) {
        sendString(content);
    }

    @Override
    public void setCoordinates(ControllerCoords c) {
        final float nx = c.channel0;
        final float ny = c.channel1;
        final boolean hasCh0 = nx != ControllerCoords.UNDEFINED;
        final boolean hasCh1 = ny != ControllerCoords.UNDEFINED;
        if (hasCh0 && hasCh1) {
            // decompose the motion
            final float left = -ny * (nx > 0 ? (-nx + 1) : 1);
            final float right = -ny * (nx < 0 ? (nx + 1) : 1);
            sendEarsMotion(left, right);
        }
    }


    public void doResume() {
    }

    public void doPause() {
    }

    public void doTeardown() {
        closeCurrentConnection();
        if (mBtReceiverRegistered) {
            mContext.unregisterReceiver(mReceiver);
            mBtReceiverRegistered = false;
        }
    }

    private void closeCurrentConnection() {
        if (mBluetoothRemoteDevice != null) {
            mBluetoothRemoteDevice = null;
        }
        if (mConnectThread != null) {
            mConnectThread.cancel();
            mConnectThread = null;
        }
        if (mConnectedThread != null) {
            mConnectedThread.cancel();
            mConnectedThread = null;
        }
    }

    /* private stuff ahead */

    private void connectToRemoteDevice() {
        if (mBluetoothRemoteDevice == null) {
            Logger.userVisibleMessage("BluetoothLink.connectToRemoteDevice: no remote device! fixme.");
            return;
        }
        mBluetoothAdapter.cancelDiscovery();
        if (mConnectThread != null)
            mConnectThread.cancel();
        mConnectThread = new ConnectThread(mBluetoothRemoteDevice);
        mConnectThread.start();
    }

    private class ConnectThread extends Thread {
        private final BluetoothSocket mSocket;

        public ConnectThread(BluetoothDevice device) {
            BluetoothSocket tmp = null;

            // Get a BluetoothSocket to connect with the given BluetoothDevice
            try {
                tmp = mBluetoothRemoteDevice.createRfcommSocketToServiceRecord(UUID.fromString(BT_SERIAL_BOARD_UUID));
            } catch (IOException e) {
                Logger.exception("BluetoothLink.connectToRemoteDevice: error creating Rfcomm socket to "+ mBtTargetDeviceName, e);
            }
            mSocket = tmp;
        }

        @Override
        public void run() {
            try {
                // Connect the device through the socket. This will block until it succeeds or throws an exception
                mSocket.connect();
            } catch (IOException connectException) {
                // Unable to connect; close the socket and get out
                Logger.exception("BluetoothLink.connectToRemoteDevice: error connecting the socket", connectException);
                try {
                    mSocket.close();
                } catch (IOException closeException) {
                    Logger.exception("BluetoothLink.connectToRemoteDevice: error cleaning up the socket", closeException);
                }
                return;
            }

            // Do work to manage the connection (in a separate thread)
            Logger.wtf("CONNECTED !!!!!");
            mConnectedThread = new ConnectedThread(mSocket);
            mConnectedThread.start();
        }

        /** Will cancel an in-progress connection, and close the socket */
        public void cancel() {
            try {
                if (mSocket != null)
                    mSocket.close();
            } catch (IOException e) { }
        }
    }

    private class ConnectedThread extends Thread {
        private final BluetoothSocket mmSocket;
        private final InputStream mmInStream;
        private final OutputStream mmOutStream;

        public ConnectedThread(BluetoothSocket socket) {
            mmSocket = socket;
            InputStream tmpIn = null;
            OutputStream tmpOut = null;

            // Get the input and output streams, using temp objects because
            // member streams are final
            try {
                tmpIn = socket.getInputStream();
                tmpOut = socket.getOutputStream();
            } catch (IOException e) {
                Logger.exception("BluetoothLink.connectToRemoteDevice: can't get streams to ConnectedThread", e);
            }

            mmInStream = tmpIn;
            mmOutStream = tmpOut;
        }

        @Override
        public void run() {
            byte[] buffer = new byte[1024];  // buffer store for the stream
            int bytes; // bytes returned from read()

            // Keep listening to the InputStream until an exception occurs
            while (true) {
                try {
                    // Read from the InputStream
                    bytes = mmInStream.read(buffer);
                    // Send the obtained bytes to the UI activity
                    ////mHandler.obtainMessage(MESSAGE_READ, bytes, -1, buffer)
                    ////        .sendToTarget();
                    // BUT HERE WE HAVE NOTHING INCOMING .. SO ...
                } catch (IOException e) {
                    break;
                }
            }
        }

        /* Call this from the main activity to send data to the remote device */
        public void write(byte[] bytes, int offset, int count) {
            try {
                mmOutStream.write(bytes, offset, count);
            } catch (IOException e) {
                Logger.exception("BluetoothLink.write: error writing the packet", e);
            }
        }

        /* Call this from the main activity to shutdown the connection */
        public void cancel() {
            try {
                mmSocket.close();
            } catch (IOException e) { }
        }
    }

    private BluetoothDevice findBondedDeviceByName(String visibleName) {
        if (!mIsSupported)
            return null;
        for (BluetoothDevice device : mBluetoothAdapter.getBondedDevices()) {
            if (visibleName.equals(device.getName()))
                return device;
        }
        Logger.userVisibleMessage("Not paired at startup");
        return null;
    }

    private final BroadcastReceiver mReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();

            if (BluetoothAdapter.ACTION_STATE_CHANGED.equals(action)) {
                Logger.wtf("BluetoothLink.mReceiver: ACTION_STATE_CHANGED to "+intent.getIntExtra(BluetoothAdapter.EXTRA_STATE, -1)+", from "+intent.getIntExtra(BluetoothAdapter.EXTRA_PREVIOUS_STATE, -1));
            }
            // called for each device that has been found after a scan
            else if (BluetoothDevice.ACTION_FOUND.equals(action)) {
                // Get the BluetoothDevice object from the Intent
                BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                if (mBtTargetDeviceName.equals(device.getName())) {
                    if (mBluetoothRemoteDevice != null) {
                        Logger.userVisibleMessage("BluetoothLink: ignoring found again: "+ mBtTargetDeviceName);
                        return;
                    }
                    Logger.userVisibleMessage("BluetoothLink: scanned and found "+ mBtTargetDeviceName);
                    mBluetoothRemoteDevice = device;
                    // -> Connect to it!
                    connectToRemoteDevice();
                }
            }
            else if (BluetoothDevice.ACTION_ACL_CONNECTED.equals(action)) {
                // Get the BluetoothDevice object from the Intent
                BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                Logger.userVisibleMessage("BT Target Device Connected");
                mIsConnected = true;
                if (mListener != null)
                    mListener.btConnectionChanged(true);
                // TODO: robustness
            }
            // called if/when the device is disconnected
            else if (BluetoothDevice.ACTION_ACL_DISCONNECTED.equals(action)) {
                // Get the BluetoothDevice object from the Intent
                BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                if (mBluetoothRemoteDevice != null && device != null && device.equals(mBluetoothRemoteDevice)) {
                    // known device
                    Logger.userVisibleMessage("Bluetooth Device Disconnected");
                    mIsConnected = false;
                    if (mListener != null)
                        mListener.btConnectionChanged(false);
                    closeCurrentConnection();
                } else {
                    // unknown device
                    //Logger.userVisibleMessage("Unknown Bluetooth Device Disconnected");
                    //Logger.apierror("BluetoothLink: Unk FIXME!");
                }
            }
            else
                Logger.wtf("BluetoothLink.mReceiver: unhandled action "+intent.getAction()+". Fixme.");
        }
    };

    /**
     * Controls the motion of the ears
     *
     * @param leftEar left ear; -1: max back: 0: stop, 1: max forward
     * @param rightEar right ear; -1: max back: 0: stop, 1: max forward
     */
    private void sendEarsMotion(float leftEar, float rightEar) {
        leftEar = floatSuppress(-0.05f, floatBound(-1f, leftEar, 1f), 0.05f, 0);
        rightEar = floatSuppress(-0.05f, floatBound(-1f, rightEar, 1f), 0.05f, 0);

        final int leftVal = Math.round(leftEar * 100f + 100f);
        final int rightVal = Math.round(rightEar * 100f + 100f);

        sendCommand4((byte)0x02, (byte)0x01, leftVal, rightVal);
    }


    /* Private stuff ahead */

    private float floatBound(float min, float val, float max) {
        if (val > max)
            return max;
        if (val < min)
            return min;
        return val;
    }

    private float floatSuppress(float from, float val, float to, float dest) {
        if (val > from && val < to)
            return dest;
        return val;
    }

    /*private boolean sendCommand3(byte command, byte target, int value) {
        return sendCommand4(command, target, value, 0);
    }*/

    private boolean sendString(String string) {
        if (mConnectedThread != null) {
            byte[] b = string.getBytes(StandardCharsets.US_ASCII);
            mConnectedThread.write(b, 0, b.length);
            return true;
        }
        return false;
    }

    private boolean sendCommand4(byte command, byte target, int value1, int value2) {
        if (value1 > 255)
            value1 = 255;
        if (value2 > 255)
            value2 = 255;

        mSendBuffer[0] = (byte) 0xA5;
        mSendBuffer[1] = command;
        mSendBuffer[2] = target;
        mSendBuffer[3] = (byte) value1;
        mSendBuffer[4] = (byte) value2;

        if (mConnectedThread != null) {
            mConnectedThread.write(mSendBuffer, 0, COMM_PACKET_SIZE);
            return true;
        }
        return false;
    }

}
