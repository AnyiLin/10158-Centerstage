package org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration;

import java.util.ArrayList;

public class PathChain {
    private ArrayList<Path> pathChain = new ArrayList<>();

    private ArrayList<PathCallback> callbacks = new ArrayList<>();

    /**
     * This creates a new path chain from some specified paths
     *
     * IMPORTANT NOTE: Order matters here. The order in which the paths are input is the order in
     * which they will be run.
     *
     * @param paths the specified paths
     */
    public PathChain(Path... paths) {
        for (Path path : paths) {
            pathChain.add(path);
        }
    }

    /**
     * This creates a new path chain from an ArrayList of specified paths
     *
     * IMPORTANT NOTE: Order matters here. The order in which the paths are input is the order in
     * which they will be run.
     *
     * @param paths the ArrayList of specified paths
     */
    public PathChain(ArrayList<Path> paths) {
        pathChain = paths;
    }

    /**
     * This returns the path on the path chain at a specified index
     *
     * @param index the index specified
     * @return returns the path at the index
     */
    public Path getPath(int index) {
        return pathChain.get(index);
    }

    /**
     * This returns the size of the path chain
     *
     * @return returns the size of the path chain
     */
    public int size() {
        return pathChain.size();
    }

    public void setCallbacks(PathCallback... callbacks) {
        for (PathCallback callback : callbacks) {
            this.callbacks.add(callback);
        }
    }

    public void setCallbacks(ArrayList<PathCallback> callbacks) {
        this.callbacks = callbacks;
    }

    public ArrayList<PathCallback> getCallbacks() {
        return callbacks;
    }
}
