# 2022-to-2024-migration

## Setting Up

### fixing junit unresolved dependency
```
Invoke-WebRequest -Uri https://repo.maven.apache.org/maven2/org/junit/jupiter/junit-jupiter/5.10.1/junit-jupiter-5.10.1.module -OutFile C:\Users\Public\wpilib\2024\maven\org\junit\jupiter\junit-jupiter\5.10.1\junit-jupiter-5.10.1.module
Invoke-WebRequest -Uri https://repo.maven.apache.org/maven2/org/junit/junit-bom/5.10.1/junit-bom-5.10.1.module -OutFile C:\Users\Public\wpilib\2024\maven\org\junit\junit-bom\5.10.1\junit-bom-5.10.1.module
```
run vscode `Clean Java Language Server Workspace` command

https://docs.wpilib.org/en/stable/docs/yearly-overview/known-issues.html

### setting up github package repository

create a file called secrets.json and add this text

```
{
    "github_package_auth" : {
        "username" : "my_username", 
        "personal_access_token" : "github_pat_redacted"
    }
}
```
replace `my_username` with your github username

replace `github_pat_redacted` with your github personal access token with readonly permission to all public repos

https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens#creating-a-fine-grained-personal-access-token