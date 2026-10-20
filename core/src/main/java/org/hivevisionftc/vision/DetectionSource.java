package org.hivevisionftc.vision;

import androidx.annotation.NonNull;

import java.util.List;

public interface DetectionSource {
    @NonNull
    List<Detection> latest();

    long stalenessMs();
}
