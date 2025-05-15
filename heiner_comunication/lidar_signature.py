import math

def get_lidar_signature(lidar) -> list:
    """
    Ermittelt die Lidar Signatur in 8 Richtungen.
    Rückgabe: Liste von 8 Distanzen.
    Reihenfolge:
    [Front, leicht links, links, hinten links, hinten, hinten rechts, rechts, leicht rechts]
    """
    directions_deg = [0, 45, 90, 135, 180, -135, -90, -45]
    signature = []
    tolerance = math.radians(22.5)  # ±22.5° für 8 Bereiche

    for deg in directions_deg:
        angle = math.radians(deg)
        distance = lidar.get_value_around_angle_min(angle, tolerance)
        signature.append(distance)

    return signature

def rotate_signature(signature, steps) -> list:
    """
    Rotiert die Signatur um n Schritte (links positiv).
    steps = 1 → um 45° links drehen.
    steps = -1 → um 45° rechts drehen.
    """
    steps = steps % len(signature)
    return signature[steps:] + signature[:steps]

def compare_signatures(sig1, sig2) -> float:
    """
    Vergleicht zwei Signaturen.
    Gibt die durchschnittliche absolute Differenz zurück.
    """
    assert len(sig1) == len(sig2), "Signaturen müssen gleich lang sein."
    differences = [abs(a - b) for a, b in zip(sig1, sig2)]
    avg_difference = sum(differences) / len(differences)
    return avg_difference
