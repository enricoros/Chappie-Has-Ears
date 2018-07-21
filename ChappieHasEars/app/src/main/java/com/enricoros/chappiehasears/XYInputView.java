package com.enricoros.chappiehasears;

import android.content.Context;
import android.content.res.TypedArray;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

public class XYInputView extends View {

    private final static float UNDEFINED = -2;

    private final Paint mPtPaint;
    private final Paint mCtPaint;

    private float mLastNx = UNDEFINED;
    private float mLastNy = UNDEFINED;
    private boolean mAvoidFeedback;

    public XYInputView(Context context, AttributeSet attrs) {
        super(context, attrs);

        mPtPaint = new Paint();
        TypedArray themeColors = context.getTheme().obtainStyledAttributes(new int[]{android.R.attr.colorAccent,});
        mPtPaint.setColor(themeColors.getColor(0, Color.BLUE));
        mPtPaint.setAntiAlias(true);

        mCtPaint = new Paint();
        mCtPaint.setColor(Color.WHITE);
        mCtPaint.setAntiAlias(true);
    }

    interface Listener {
        void onNewPoint(float nx, float ny);
    }

    private Listener mListener;

    public void setListener(Listener listener) {
        mListener = listener;
    }

    /**
     * @param nx In the -1 ... 1 range
     * @param ny In the -1 ... 1 range
     */
    public void setPoint(float nx, float ny) {
        if (mAvoidFeedback)
            return;
        if (mLastNx != nx || mLastNy != ny) {
            mLastNx = nx;
            mLastNy = ny;
            invalidate();
        }
    }

    public void disablePoint() {
        setPoint(UNDEFINED, UNDEFINED);
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        final int W = getWidth();
        final int H = getHeight();

        mLastNx = normate(event.getX(), W);
        mLastNy = normate(event.getY(), H);
        if (mListener != null) {
            mAvoidFeedback = true;
            mListener.onNewPoint(mLastNx, mLastNy);
            mAvoidFeedback = false;
        }

        invalidate();
        return true;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        final int W = getWidth();
        final int H = getHeight();

        if (mLastNx != UNDEFINED && mLastNy != UNDEFINED) {
            final int cx = expand(mLastNx, W);
            final int cy = expand(mLastNy, H);
            canvas.drawCircle(cx, cy, 24, mPtPaint);
            canvas.drawLine(cx, 0, cx, H, mCtPaint);
            canvas.drawLine(0, cy, W, cy, mCtPaint);
        }

        canvas.drawCircle(expand(0, W), expand(0, H), 8, mCtPaint);
    }

    private float normate(float expandedVal, float scale) {
        float v = 2.0f * expandedVal / scale - 1;
        return v > -1 ? (v < 1 ? v : 1) : -1;
    }

    private int expand(float normVal, float scale) {
        return Math.round((normVal + 1) / 2f * scale);
    }

}
