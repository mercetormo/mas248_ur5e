import json
import sys
from pathlib import Path


DEFAULT_TARGETS = Path("/tmp/mas248_targets.json")


def load_targets(path):
    """Charge et valide les cibles générées par image_to_targets.py."""
    if not path.is_file():
        raise FileNotFoundError(
            f"Fichier de cibles introuvable : {path}\n"
            "Génère d'abord les cibles avec image_to_targets.py."
        )

    with path.open("r", encoding="utf-8") as file:
        data = json.load(file)

    if not isinstance(data, dict):
        raise ValueError("Le JSON doit contenir un objet.")

    strokes = data.get("strokes")
    if not isinstance(strokes, list) or not strokes:
        raise ValueError(
            "Aucun trait trouvé dans le fichier JSON."
        )

    for index, stroke in enumerate(strokes, start=1):
        if not isinstance(stroke, dict):
            raise ValueError(f"Le trait {index} n'est pas valide.")

        poses = stroke.get("poses")
        if not isinstance(poses, list) or not poses:
            raise ValueError(
                f"Le trait {index} ne contient aucune pose."
            )

        for pose_index, pose in enumerate(poses, start=1):
            required = ("x", "y", "z", "qx", "qy", "qz", "qw")

            if not isinstance(pose, dict):
                raise ValueError(
                    f"Pose {pose_index} du trait {index} invalide."
                )

            missing = [key for key in required if key not in pose]
            if missing:
                raise ValueError(
                    f"Pose {pose_index} du trait {index} : "
                    f"champs manquants : {', '.join(missing)}"
                )

            for key in required:
                value = pose[key]
                if isinstance(value, bool) or not isinstance(
                    value, (int, float)
                ):
                    raise ValueError(
                        f"Valeur invalide pour '{key}', "
                        f"trait {index}, pose {pose_index}."
                    )

    return data


def main():
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_TARGETS

    try:
        data = load_targets(path)
    except (OSError, json.JSONDecodeError, ValueError) as error:
        print(f"Erreur : {error}", file=sys.stderr)
        sys.exit(1)

    strokes = data["strokes"]
    total_poses = sum(len(stroke["poses"]) for stroke in strokes)

    print("Fichier de cibles chargé avec succès.")
    print(f"Fichier : {path}")
    print(f"Nombre de traits : {len(strokes)}")
    print(f"Nombre total de poses : {total_poses}")

    for index, stroke in enumerate(strokes, start=1):
        poses = stroke["poses"]
        first = poses[0]
        last = poses[-1]

        print(
            f"Trait {index} : {len(poses)} poses | "
            f"début=({first['x']:.3f}, {first['y']:.3f}, "
            f"{first['z']:.3f}) | "
            f"fin=({last['x']:.3f}, {last['y']:.3f}, "
            f"{last['z']:.3f})"
        )

    print("Mode vérification uniquement : aucun mouvement commandé.")


if __name__ == "__main__":
    main()
