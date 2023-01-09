from bagpy import bagreader
import pandas as pd
import numpy as np

b = bagreader("data/2022-10-31-15-05-16.bag")
print(b.topic_table)

chatter_csv = b.message_by_topic("/chatter")
df = pd.read_csv(chatter_csv)
print(df["header.frame_id"].unique())

chatter_csv = b.message_by_topic("/vrpn_client_node/awww/pose")
df = pd.read_csv(chatter_csv)
print(df)
