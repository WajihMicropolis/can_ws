
# Managing Submodules in Your Git Repository

This guide will help you add, pull, clone, and push updates with submodules in your Git repository.

## Adding a Submodule

To add the `CAN_WS` repository as a submodule, use the following command:

```sh
git submodule add https://github.com/WajihMicropolis/can_ws
```

This command will create a `.gitmodules` file and pull the `can_ws` repository into your branch.

## Pulling with Submodules

If you want to pull your branch along with its submodules, use:

```sh
git pull --recurse-submodules
```

You can also update your Git configuration to automatically include `--recurse-submodules` in every pull:

```sh
git config submodule.recurse true
```

## Cloning the Repository with Submodules

When cloning a repository with submodules, first use the `git clone` command:

```sh
git clone <repository_url>
```

Then, initialize and update the submodules with:

```sh
git submodule update --init
```

## Pushing Updates from Submodules

To push updates from submodules, follow these steps:

1. Navigate to the submodule directory:

    ```sh
    cd path/to/submodule
    ```

2. Commit your changes in the submodule:

    ```sh
    git add .
    git commit -m "Your commit message"
    git push origin <branch>
    ```

3. Go back to the main repository and commit the submodule update:

    ```sh
    cd ..
    git add path/to/submodule
    git commit -m "Updated submodule"
    git push origin <branch>
    ```

