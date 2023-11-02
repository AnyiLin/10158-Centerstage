package org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration;

import java.util.ArrayList;

public class BezierCurveChain {
    private ArrayList<BezierCurve> chain;

    private final double COLINEAR_ERROR = 0.0001;

    /**
     * Fills the chain ArrayList with the provided curves
     *
     * @param curves the curves to be added
     */
    public BezierCurveChain(BezierCurve... curves) {
        for (BezierCurve curve : curves) {
            chain.add(curve);
            if (chain.size()>1) {
                // Gets important points along the previous and current curves
                Point prevSecondLast = chain.get(chain.size()-2).getSecondToLastControlPoint();
                Point prevLast = chain.get(chain.size()-2).getLastControlPoint();
                Point currentFirst = chain.get(chain.size()-1).getFirstControlPoint();
                Point currentSecond = chain.get(chain.size()-1).getSecondControlPoint();

                // Checks if start/end points are the same
                if (!(prevLast.getX() == currentFirst.getX() && prevLast.getY() == currentFirst.getY())) {
                    try {
                        throw new Exception("Curves " + (chain.size()-1) + " and " + (chain.size()-2) + " do not share start/end points");
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }

                // Checks if previous and current derivatives are the same
                if ((((currentSecond.getY() - currentFirst.getY()) / (currentSecond.getX() - currentFirst.getX())) > ((prevLast.getY() - prevSecondLast.getY()) / (prevLast.getX() - prevSecondLast.getX())) + COLINEAR_ERROR) || (((currentSecond.getY() - currentFirst.getY()) / (currentSecond.getX() - currentFirst.getX())) < ((prevLast.getY() - prevSecondLast.getY()) / (prevLast.getX() - prevSecondLast.getX())) - COLINEAR_ERROR)) {
                    try {
                        throw new Exception("Curves " + (chain.size()-1) + " and " + (chain.size()-2) + " are not collinear");
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            }
        }
    }

    /**
     * Generates ArrayList (chain) of curves chained together
     *
     * @param curves ArrayList of Curves to chain together
     * @throws Exception If start/end points are discontinuous
     */
    public BezierCurveChain(ArrayList<BezierCurve> curves) throws Exception {
        chain = curves;
        for (int i = 0; i < curves.size() - 1; i++) {

            // Gets important points along the previous and current curves
            Point prevSecondLast = chain.get(i).getSecondToLastControlPoint();
            Point prevLast = chain.get(i).getLastControlPoint();
            Point currentFirst = chain.get(i + 1).getFirstControlPoint();
            Point currentSecond = chain.get(i + 1).getSecondControlPoint();

            // Checks if start/end points are the same
            if (!(prevLast.getX() == currentFirst.getX() && prevLast.getY() == currentFirst.getY())) {
                throw new Exception("Curves " + i + " and " + (i + i) + " do not share start/end points");
            }

            // Checks if previous and current derivatives are the same
            if (((currentSecond.getY() - currentFirst.getY()) / (currentSecond.getX() - currentFirst.getX())) != ((prevLast.getY() - prevSecondLast.getY()) / (prevLast.getX() - prevSecondLast.getX()))){
                throw new Exception("Curves " + i + " and " + (i + i) + " are not collinear");
            }
        }
    }

    /**
     * This returns the ArrayList of BezierCurve containing the chain
     *
     * @return returns the ArrayList
     */
    public ArrayList<BezierCurve> getChain(){
        return chain;
    }

    /**
     * This returns the BezierCurve at index i in the chain
     *
     * @param i The index of the curve we want
     * @return returns the BezierCurve
     */
    public BezierCurve getCurve(int i){
        return chain.get(i);
    }

    /**
     * This returns the size of the BezierCurve chain
     *
     * @return returns the size
     */
    public int getChainSize() {
        return chain.size();
    }
}