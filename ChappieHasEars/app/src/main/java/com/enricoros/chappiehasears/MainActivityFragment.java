package com.enricoros.chappiehasears;

import android.app.Activity;
import android.app.Fragment;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.RatingBar;
import android.widget.SeekBar;
import android.widget.TextView;

/**
 * A placeholder fragment containing a simple view.
 */
public class MainActivityFragment extends Fragment implements BluetoothLink.Listener {

    private static final String BT_DEVICE_NAME = "Chappie-Ears";
    private static final float INITIAL_LEFT_POS = 1f;
    private static final float INITIAL_RIGHT_POS = 1f;

    private BluetoothLink mBTLink;

    // references to objects
    private RatingBar m_ratingBar;
    private XYInputView mXYInput;
    private SeekBar mLeftSeekBar;
    private SeekBar mRightSeekBar;

    // state
    private float mLeftEarPos = 0;
    private float mRightEarPos = 0;

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
    public void onStop() {
        if (mBTLink != null) {
            Logger.userVisibleMessage("Stopped BT");
            mBTLink.doTeardown();
            mBTLink = null;
        }
        super.onStop();
    }

    @Override
    public void onBTConnectionChanged(boolean connected) {
        if (m_ratingBar != null)
            m_ratingBar.setRating(connected ? 5 : 0.5f);
        if (mLeftSeekBar != null)
            mLeftSeekBar.setEnabled(connected);
        if (mRightSeekBar != null)
            mRightSeekBar.setEnabled(connected);
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.fragment_main, container, false);

        m_ratingBar = rootView.findViewById(R.id.ratingBar);

        mXYInput = rootView.findViewById(R.id.tracerView);
        mXYInput.setListener(mXyInputListener);

        mLeftSeekBar = rootView.findViewById(R.id.seekBarLeft);
        mLeftSeekBar.setOnSeekBarChangeListener(mSeekBarsChangeListener);
        mRightSeekBar = rootView.findViewById(R.id.seekBarRight);
        mRightSeekBar.setOnSeekBarChangeListener(mSeekBarsChangeListener);

        // long click to clear the log
        rootView.findViewById(R.id.msgLog).setOnLongClickListener(view -> {
            ((TextView) view).setText("");
            return true;
        });

        // demo / option buttons
        /*final CompoundButton.OnCheckedChangeListener onCheckedChangeListener = new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (mBTLink != null) {
                    int index = (Integer) buttonView.getTag();
                    mBTLink.sendFlagNumeric(index, isChecked ? 1 : 0);
                }
            }
        };
        final CheckBox checkDemo = (CheckBox) rootView.findViewById(R.id.checkDemo);
        checkDemo.setTag((Integer) 1);
        checkDemo.setOnCheckedChangeListener(onCheckedChangeListener);
        final CheckBox checkOpt1 = (CheckBox) rootView.findViewById(R.id.checkOpt1);
        checkOpt1.setTag((Integer) 2);
        checkOpt1.setOnCheckedChangeListener(onCheckedChangeListener);*/

        // modes
        final View.OnClickListener modeClick = v -> {
            int mode = Integer.parseInt((String) v.getTag());
            if (mBTLink != null)
                mBTLink.sendFlagNumeric(4, mode);
        };
        rootView.findViewById(R.id.modeA1).setOnClickListener(modeClick);
        rootView.findViewById(R.id.modeA2).setOnClickListener(modeClick);
        rootView.findViewById(R.id.modeDemo).setOnClickListener(modeClick);
        rootView.findViewById(R.id.modeStand).setOnClickListener(modeClick);
        rootView.findViewById(R.id.modeSit).setOnClickListener(modeClick);

        // edit box buttons, to send strings
        /*View sendButton = rootView.findViewById(R.id.sendButton1);
        sendButton.setOnClickListener(mSendListener);
        sendButton.setTag("LF");
        rootView.findViewById(R.id.sendButton2).setOnClickListener(mSendListener);*/

        updateCurrentPosition(INITIAL_LEFT_POS, INITIAL_RIGHT_POS);
        m_ratingBar.setOnClickListener(mDefaultInputListener);
        rootView.findViewById(R.id.textCool).setOnClickListener(v -> {
            final Activity activity = getActivity();
            activity.finish();
            Logger.userVisibleMessage("Goodbye");
        });

        return rootView;
    }


    // motion related part

    private final XYInputView.Listener mXyInputListener = (nx, ny) -> {
        // decompose the motion and update the UI
        updateCurrentPosition(
                -ny * (nx > 0 ? (-nx + 1) : 1),
                -ny * (nx < 0 ? (nx + 1) : 1)
        );
    };

    private final SeekBar.OnSeekBarChangeListener mSeekBarsChangeListener = new SeekBar.OnSeekBarChangeListener() {
        @Override
        public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
            if (fromUser) {
                if (seekBar == mLeftSeekBar)
                    updateCurrentPosition(((float) progress - 100f) / 100f, mRightEarPos);
                else
                    updateCurrentPosition(mLeftEarPos, ((float) progress - 100f) / 100f);
            }
        }

        @Override
        public void onStartTrackingTouch(SeekBar seekBar) {
        }

        @Override
        public void onStopTrackingTouch(SeekBar seekBar) {
        }
    };

    private final View.OnClickListener mDefaultInputListener = v -> updateCurrentPosition(INITIAL_LEFT_POS, INITIAL_RIGHT_POS);

    private void updateCurrentPosition(float leftPos, float rightPos) {
        final boolean leftChanged = leftPos != mLeftEarPos;
        final boolean rightChanged = rightPos != mRightEarPos;
        if (!leftChanged && !rightChanged)
            return;
        mLeftEarPos = leftPos;
        mRightEarPos = rightPos;

        if (leftChanged && mLeftSeekBar != null)
            mLeftSeekBar.setProgress(Math.round(mLeftEarPos * 100f + 100f));

        if (rightChanged && mRightSeekBar != null)
            mRightSeekBar.setProgress(Math.round(mRightEarPos * 100f + 100f));

        if (mXYInput != null) {
            if (mLeftEarPos == 1f && mRightEarPos == 1f)
                mXYInput.setPoint(0, -1);
            // don't know how to map the rest for now...
        }

        deferTransmission(leftChanged, rightChanged);
    }

    // delayed transmission (rate limiter) related stuff
    private static final int TRANSMISSION_DELAY_MS = 30;
    private boolean mDtLeftChanged = false;
    private boolean mDtRightChanged = false;
    private boolean mDtPending = false;

    private void deferTransmission(boolean leftChanged, boolean rightChanged) {
        mDtLeftChanged |= leftChanged;
        mDtRightChanged |= rightChanged;
        if (!mDtPending) {
            // m_ratingBar is just a View instance, which has an Handler...
            m_ratingBar.postDelayed(mDeferredTransmission, TRANSMISSION_DELAY_MS);
            mDtPending = true;
        }
    }

    private final Runnable mDeferredTransmission = new Runnable() {
        @Override
        public void run() {
            if (mBTLink != null) {
                if (mDtLeftChanged && mDtRightChanged)
                    mBTLink.sendEarsPosition(mLeftEarPos, mRightEarPos);
                else if (mDtLeftChanged)
                    mBTLink.sendEarPosition(1, mLeftEarPos);
                else
                    mBTLink.sendEarPosition(2, mRightEarPos);
            }
            mDtPending = false;
        }
    };


    // temp debugging box
    /*private View.OnClickListener mSendListener = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            String content = ((EditText) v.getRootView().findViewById(R.id.editSerialComm)).getText().toString();
            if ("LF".equals(v.getTag()))
                content += "\n";
            if (mBTLink != null)
                mBTLink.sendMessage(content);
        }
    };*/

}
