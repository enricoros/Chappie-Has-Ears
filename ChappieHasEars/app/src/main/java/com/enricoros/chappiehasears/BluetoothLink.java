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

public class BluetoothLink {

    private final static String BT_SERIAL_BOARD_UUID = "00001101-0000-1000-8000-00805f9b34fb";

    private final Context mContext;
    private final String mBtTargetDeviceName;
    private final BluetoothAdapter mBluetoothAdapter;
    private final boolean mIsSupported;

    private boolean mBtReceiverRegistered;
    private BluetoothDevice mBluetoothRemoteDevice;

    private boolean mIsConnected;
    private ConnectionThread mConnectionThread;
    private CommunicationThread mCommunicationThread;

    private static final int COMM_PACKET_SIZE = 5;
    private final byte[] mSendBuffer;

    private Listener mListener;

    interface Listener {
        void onBTConnectionChanged(boolean connected);
    }

    BluetoothLink(Context context, String btDeviceName) {
        mSendBuffer = new byte[COMM_PACKET_SIZE];
        mContext = context;
        mBtTargetDeviceName = btDeviceName;

        // check bluetooth_availability and enablement
        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        mIsSupported = mBluetoothAdapter != null && mBluetoothAdapter.isEnabled();
        if (!mIsSupported) {
            Logger.userVisibleMessage("Bluetooth can not be used. Please enable it and restart this APP.");
            //Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            //startActivityForResult(enableBtIntent, Activity.REQUEST_ENABLE_BT);
            return;
        }

        // active and enabled
        IntentFilter btIntentFilter = new IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED);
        btIntentFilter.addAction(BluetoothDevice.ACTION_FOUND);
        btIntentFilter.addAction(BluetoothDevice.ACTION_BOND_STATE_CHANGED);
        btIntentFilter.addAction(BluetoothDevice.ACTION_ACL_CONNECTED);
        btIntentFilter.addAction(BluetoothDevice.ACTION_ACL_DISCONNECTED);
        mContext.registerReceiver(mBtNotifications, btIntentFilter);
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

    public boolean getIsSupported() {
        return mIsSupported;
    }

    void setListener(Listener listener) {
        mListener = listener;
        if (mListener != null)
            mListener.onBTConnectionChanged(mIsConnected);
    }


    /**
     * @param earIndex 1..N
     * @param value    -1: max mack .. 1: max forward
     */
    void sendEarPosition(int earIndex, float value) {
        sendCommand4((byte) 0x01, (byte) earIndex,
                marshallNormalizePosition(value), 0);
    }

    /**
     * Controls the motion of the ears
     *
     * @param leftEar  left ear; -1: max back: 0: stop, 1: max forward
     * @param rightEar right ear; -1: max back: 0: stop, 1: max forward
     */
    void sendEarsPosition(float leftEar, float rightEar) {
        sendCommand4((byte) 0x02, (byte) 0x01,
                marshallNormalizePosition(leftEar),
                marshallNormalizePosition(rightEar));
    }

    /**
     * Sets an application dependent flag (not that the rest is not app dependent...)
     */
    void sendFlagNumeric(int index, int flag) {
        sendCommand4((byte) 0x04, (byte) index, (byte) flag, 0);
    }

    /**
     * @param message The ASCII representation of the message will be sent
     */
    public void sendMessage(String message) {
        sendString(message);
    }


    /*** private stuff ahead ***/

    void doTeardown() {
        closeCurrentConnection();
        if (mBtReceiverRegistered) {
            mContext.unregisterReceiver(mBtNotifications);
            mBtReceiverRegistered = false;
        }
    }

    private void closeCurrentConnection() {
        if (mBluetoothRemoteDevice != null) {
            mBluetoothRemoteDevice = null;
        }
        if (mConnectionThread != null) {
            mConnectionThread.cancel();
            mConnectionThread = null;
        }
        if (mCommunicationThread != null) {
            mCommunicationThread.cancel();
            mCommunicationThread = null;
        }
    }

    private void connectToRemoteDevice() {
        if (mBluetoothRemoteDevice == null) {
            Logger.userVisibleMessage("BluetoothLink.connectToRemoteDevice: no remote device! fixme.");
            return;
        }
        mBluetoothAdapter.cancelDiscovery();
        if (mConnectionThread != null)
            mConnectionThread.cancel();
        mConnectionThread = new ConnectionThread(mBluetoothRemoteDevice);
        mConnectionThread.start();
    }

    private class ConnectionThread extends Thread {
        private final BluetoothSocket mSocket;

        ConnectionThread(BluetoothDevice device) {
            BluetoothSocket tmp = null;

            final int bondState = device.getBondState();
            switch (bondState) {
                case BluetoothDevice.BOND_BONDED:
                    Logger.info(device.getName() + " is already bond");
                    break;
                case BluetoothDevice.BOND_NONE:
                    Logger.info("Not yet bond as BT device");
                    break;
                case BluetoothDevice.BOND_BONDING:
                    Logger.info("Bonding...");
                    break;
            }

            // Get a BluetoothSocket to connect with the given BluetoothDevice
            try {
                tmp = device.createRfcommSocketToServiceRecord(UUID.fromString(BT_SERIAL_BOARD_UUID));
            } catch (IOException e) {
                Logger.exception("BluetoothLink.connectToRemoteDevice: error creating Rfcomm socket to " + mBtTargetDeviceName, e);
            }
            mSocket = tmp;
        }

        @Override
        public void run() {
            try {
                // Connect the device through the socket. This will block until it succeeds or throws an exception
                mSocket.connect();
                // Connected, now start the communication
                mCommunicationThread = new CommunicationThread(mSocket);
                mCommunicationThread.start();
            } catch (IOException connectException) {
                // Unable to connect; close the socket and get out
                Logger.exception("BluetoothLink.connectToRemoteDevice: error connecting the socket", connectException);
                try {
                    mSocket.close();
                } catch (IOException closeException) {
                    Logger.exception("BluetoothLink.connectToRemoteDevice: error cleaning up the socket", closeException);
                }
            }

            // Notify listeners
            Logger.info(mSocket.isConnected() ? "Connected" : "Socket not connected");
            setIsConnected(mSocket.isConnected());
        }

        /**
         * Will cancel an in-progress connection, and close the socket
         */
        void cancel() {
            try {
                if (mSocket != null)
                    mSocket.close();
            } catch (IOException ignored) {
            }
        }
    }

    private void setIsConnected(boolean connected) {
        mIsConnected = connected;
        if (mListener != null && mContext != null)
            ((MainActivity) mContext).runOnUiThread(() -> mListener.onBTConnectionChanged(connected));
    }

    private class CommunicationThread extends Thread {
        private final BluetoothSocket mmSocket;
        private final InputStream mmInStream;
        private final OutputStream mmOutStream;

        CommunicationThread(BluetoothSocket socket) {
            mmSocket = socket;
            InputStream tmpIn = null;
            OutputStream tmpOut = null;

            // Get the input and output streams, using temp objects because
            // member streams are final
            try {
                tmpIn = socket.getInputStream();
                tmpOut = socket.getOutputStream();
            } catch (IOException e) {
                Logger.exception("BluetoothLink.connectToRemoteDevice: can't get streams to CommunicationThread", e);
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
        void write(byte[] bytes, int offset, int count) {
            try {
                mmOutStream.write(bytes, offset, count);
            } catch (IOException e) {
                Logger.exception("BluetoothLink.write: error writing the packet", e);
            }
        }

        /* Call this from the main activity to shutdown the connection */
        void cancel() {
            try {
                mmSocket.close();
            } catch (IOException ignored) {
            }
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

    private final BroadcastReceiver mBtNotifications = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();

            if (BluetoothAdapter.ACTION_STATE_CHANGED.equals(action)) {
                Logger.wtf("BluetoothLink.mBtNotifications: ACTION_STATE_CHANGED to " + intent.getIntExtra(BluetoothAdapter.EXTRA_STATE, -1) + ", from " + intent.getIntExtra(BluetoothAdapter.EXTRA_PREVIOUS_STATE, -1));
            }
            // called for each device that has been found after a scan
            else if (BluetoothDevice.ACTION_FOUND.equals(action)) {
                // Get the BluetoothDevice object from the Intent
                BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                if (mBtTargetDeviceName.equals(device.getName())) {
                    if (mBluetoothRemoteDevice != null) {
                        Logger.userVisibleMessage("BluetoothLink: ignoring found again: " + mBtTargetDeviceName);
                        return;
                    }
                    // -> Connect to it!
                    Logger.userVisibleMessage("BluetoothLink: scanned and found " + mBtTargetDeviceName);
                    mBluetoothRemoteDevice = device;
                    connectToRemoteDevice();
                }
            } else if (BluetoothDevice.ACTION_ACL_CONNECTED.equals(action)) {
                // Get the BluetoothDevice object from the Intent
                BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                if (mBtTargetDeviceName.equals(device.getName())) {
                    // known device name: connect
                    Logger.userVisibleMessage("BT Connected - not starting new action");
//                    mBluetoothRemoteDevice = device;
//                    connectToRemoteDevice();
                }
            }
            // called if/when the device is disconnected
            else if (BluetoothDevice.ACTION_ACL_DISCONNECTED.equals(action)) {
                // Get the BluetoothDevice object from the Intent
                BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                if (device != null && device.equals(mBluetoothRemoteDevice)) {
                    // our device: disconnect
                    Logger.userVisibleMessage("Bluetooth Device Disconnected");
                    closeCurrentConnection();
                    setIsConnected(false);
                }
            } else
                Logger.wtf("BluetoothLink.mBtNotifications: unhandled action " + intent.getAction() + ". Fixme.");
        }
    };


    /*** Private stuff ahead, our Communication encoding/marshalling mechanism ***/

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

        if (mCommunicationThread != null) {
            mCommunicationThread.write(mSendBuffer, 0, COMM_PACKET_SIZE);
            return true;
        }
        return false;
    }

    private boolean sendString(String string) {
        if (mCommunicationThread != null) {
            byte[] b = string.getBytes(StandardCharsets.US_ASCII);
            mCommunicationThread.write(b, 0, b.length);
            return true;
        }
        return false;
    }

    private int marshallNormalizePosition(float np) {
        np = floatSuppress(-0.05f, floatBound(-1f, np, 1f), 0.05f, 0);
        return Math.round(np * 100f + 100f);
    }

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

}
