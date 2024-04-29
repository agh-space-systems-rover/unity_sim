import os


def find_unity_version(project_dir: str):
    # Read the version of the Unity project from ""./ProjectSettings/ProjectVersion.txt".
    with open(
        os.path.join(project_dir, "unity_sim/ProjectSettings/ProjectVersion.txt")
    ) as f:
        content = f.read()
        i = "".join(content).find("m_EditorVersion: ")
        if i == -1:
            raise RuntimeError("Failed to parse Unity project version.")
        version = content[i + len("m_EditorVersion: ") :].split("\n")[0]
        if version == "":
            raise RuntimeError("Failed to parse Unity project version.")

    return version
