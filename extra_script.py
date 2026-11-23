Import("env")
import os
import shutil

framework = env.PioPlatform().get_package_dir("framework-arduinoadafruitnrf52")
src = os.path.join(env["PROJECT_DIR"], "variants", "promicro")
dst = os.path.join(framework, "variants", "promicro")
if os.path.isdir(dst):
    shutil.rmtree(dst)
shutil.copytree(src, dst)
print("Installed board variant:", dst)
