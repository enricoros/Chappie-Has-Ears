package com.enricoros.chappiehasears;

import android.app.Fragment;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.EditText;
import android.widget.RatingBar;

/**
 * A placeholder fragment containing a simple view.
 */
public class MainActivityFragment extends Fragment implements BluetoothLink.Listener {

    public static final String BT_DEVICE_NAME = "Chappie-Ears";

    private BluetoothLink mBTLink;
    private RatingBar m_ratingBar;

    public MainActivityFragment() {
    }


    @Override
    public void onStart() {
        super.onStart();
        if (mBTLink == null) {
            mBTLink = new BluetoothLink(getActivity(), BT_DEVICE_NAME);
            mBTLink.setListener(this);
        }
    }

    @Override
    public void onResume() {
        super.onResume();
        if (mBTLink != null)
            mBTLink.doResume();
    }

    @Override
    public void onPause() {
        if (mBTLink != null)
            mBTLink.doPause();
        super.onPause();
    }

    @Override
    public void onStop() {
        if (mBTLink != null) {
            mBTLink.doTeardown();
            mBTLink = null;
        }
        super.onStop();
    }


    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.fragment_main, container, false);

        View sendButton = rootView.findViewById(R.id.sendButton1);
        sendButton.setOnClickListener(mSendListener);
        sendButton.setTag("LF");
        rootView.findViewById(R.id.sendButton2).setOnClickListener(mSendListener);

        m_ratingBar = (RatingBar) rootView.findViewById(R.id.ratingBar);

        XYInputView xyInputView = (XYInputView) rootView.findViewById(R.id.tracerView);
        xyInputView.setListener(new XYInputView.Listener() {
            @Override
            public void onNewPoint(float nx, float ny, long timeStampMs) {
                if (mBTLink == null)
                    return;
                // TODO: project here
                ControllerCoords c = new ControllerCoords(nx, ny);
                mBTLink.setCoordinates(c);
            }
        });

        return rootView;
    }

    private View.OnClickListener mSendListener = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            String content = ((EditText) v.getRootView().findViewById(R.id.editSerialComm)).getText().toString();
            if ("LF".equals(v.getTag()))
                content += "\n";
            if (mBTLink != null)
                mBTLink.writeSerialComm(content);
        }
    };

    @Override
    public void btConnectionChanged(boolean connected) {
        if (m_ratingBar != null)
            m_ratingBar.setRating(connected ? 5 : 1);
    }
}
