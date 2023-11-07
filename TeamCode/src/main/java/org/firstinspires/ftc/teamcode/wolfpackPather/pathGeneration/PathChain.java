package org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration;

import java.util.ArrayList;

public class PathChain {
    ArrayList<Path> pathChain = new ArrayList<Path>();

    public PathChain(Path... paths) {
        for (Path path : paths) {
            pathChain.add(path);
        }
    }

    public PathChain(ArrayList<Path> paths) {
        pathChain = paths;
    }

    public Path getPath(int index) {
        return pathChain.get(index);
    }

    public int size() {
        return pathChain.size();
    }
}
